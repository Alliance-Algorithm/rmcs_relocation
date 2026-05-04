#include "runtime.hpp"

#include "collector.hpp"
#include "map_descriptor_db.hpp"
#include "tools/geometry_tools.hpp"
#include "tools/numeric_tools.hpp"
#include "tools/param_tools.hpp"
#include "tools/registration_tools.hpp"
#include "tools/scan_context.hpp"
#include "validator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <numbers>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <rclcpp/qos.hpp>
#include <rmcs_msgs/srv/relocalize.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace rmcs::location {

struct RelocalizationServer::Impl {
    //在 handle_relocalize 中标记 busy，析构时自动清除。确保同一时刻只有一条重定位请求在处理。
    struct BusyGuard {
        explicit BusyGuard(Impl& impl)
            : impl_(impl) {}

        ~BusyGuard() {
            auto lock = std::scoped_lock{impl_.state_mutex_};
            impl_.busy_ = false;
        }

        Impl& impl_;
    };

    explicit Impl(RelocalizationServer& node)
        : node_(node)
        , tf_buffer_(node.get_clock())
        , tf_listener_(std::make_unique<tf2_ros::TransformListener>(tf_buffer_, &node_, false))
        , tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(&node_)) {
        load_parameters();
        initialize_modules();
        initialize_transform_cache();
        load_map();

        service_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        pointcloud_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        relocalize_service_ = node_.create_service<rmcs_msgs::srv::Relocalize>(
            service_name_,
            [this](
                const std::shared_ptr<rmcs_msgs::srv::Relocalize::Request> request,
                std::shared_ptr<rmcs_msgs::srv::Relocalize::Response> response) {
                handle_relocalize(request, std::move(response));
            },
            rclcpp::ServicesQoS(), service_group_);

        const auto publish_interval = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / std::max(1.0, publish_tf_rate_hz_)));
        tf_publish_timer_ =
            node_.create_wall_timer(publish_interval, [this] { publish_current_world_to_odom(); });
    }

    //从 ROS 参数服务器一次性加载全部运行时参数
    void load_parameters() {
        auto params = tools::load_runtime_params(node_);
        map_path_ = std::move(params.map_path);
        descriptor_path_ = std::move(params.descriptor_path);
        service_name_ = std::move(params.service_name);
        world_frame_ = std::move(params.world_frame);
        odom_frame_ = std::move(params.odom_frame);
        base_frame_ = std::move(params.base_frame);
        publish_tf_rate_hz_ = params.publish_tf_rate_hz;

        initial_runtime_config_ = std::move(params.initial_runtime_config);
        local_runtime_config_ = std::move(params.local_runtime_config);
        wide_runtime_config_ = std::move(params.wide_runtime_config);

        initial_registration_config_ = std::move(params.initial_registration_config);
        local_registration_config_ = std::move(params.local_registration_config);
        wide_registration_config_ = std::move(params.wide_registration_config);

        initial_validation_config_ = std::move(params.initial_validation_config);
        local_validation_config_ = std::move(params.local_validation_config);
        wide_validation_config_ = std::move(params.wide_validation_config);

        scan_context_config_ = params.scan_context_config;
        log_failure_details_ = params.log_failure_details;
    }

    //初始化子模块：点云收集器、验证器
    void initialize_modules() {
        collector_ = std::make_unique<Collector>(odom_frame_);
        validator_ = std::make_unique<Validator>(
            initial_validation_config_, local_validation_config_, wide_validation_config_);
    }

    void initialize_transform_cache() {
        auto initial = geometry_msgs::msg::TransformStamped{};
        initial.header.frame_id = world_frame_;
        initial.child_frame_id = odom_frame_;
        initial.transform.rotation.w = 1.0;
        initial.transform.rotation.x = 0.0;
        initial.transform.rotation.y = 0.0;
        initial.transform.rotation.z = 0.0;
        initial.transform.translation.x = 0.0;
        initial.transform.translation.y = 0.0;
        initial.transform.translation.z = 0.0;

        auto lock = std::scoped_lock{state_mutex_};
        current_world_to_odom_ = initial;
    }


    //加载 PCD 地图文件并构建 KD 树；若提供 descriptor_path 则尝试加载 SC 描述子库
    void load_map() {
        map_cloud_ = std::make_shared<PointCloud>();
        if (map_path_.empty()) {
            map_available_ = false;
            RCLCPP_WARN(
                node_.get_logger(), "map_path is empty, relocalization will be unavailable");
            return;
        }

        if (pcl::io::loadPCDFile(map_path_, *map_cloud_) != 0) {
            map_available_ = false;
            RCLCPP_ERROR(node_.get_logger(), "failed to load map from %s", map_path_.c_str());
            return;
        }

        map_kdtree_.setInputCloud(map_cloud_);
        map_kdtree_ready_ = true;
        map_available_ = true;

        RCLCPP_INFO(
            node_.get_logger(), "map loaded: %s (%zu points)", map_path_.c_str(),
            map_cloud_->size());

        load_scan_context_db();
    }

    /**
     * @brief 加载 SC 描述子库（.sc_desc）。失败则关闭 SC，wide 走 fallback。
     *
     * Phase 4：仅 bootstrap，wide handler 还不调用 db。Phase 5 接入。
     */
    void load_scan_context_db() {
        scan_context_available_ = false;
        if (descriptor_path_.empty()) {
            RCLCPP_INFO(
                node_.get_logger(),
                "descriptor_path is empty, scan_context disabled (wide will use fallback seeds)");
            return;
        }

        const auto map_hash = tools::compute_map_hash(*map_cloud_);
        if (!map_descriptor_db_.load(descriptor_path_, scan_context_config_, map_hash)) {
            RCLCPP_WARN(
                node_.get_logger(),
                "scan_context descriptor load failed (path=%s, expected_hash=0x%08x); "
                "wide will use fallback seeds",
                descriptor_path_.c_str(), map_hash);
            return;
        }

        scan_context_available_ = true;
        RCLCPP_INFO(
            node_.get_logger(),
            "scan_context descriptor loaded: %zu entries (rings=%d, sectors=%d, radius=%.2fm, "
            "hash=0x%08x)",
            map_descriptor_db_.size(), scan_context_config_.num_rings,
            scan_context_config_.num_sectors, scan_context_config_.max_radius_m, map_hash);
    }

    // 在开始处理重定位请求前，将响应对象重置为默认的失败状态
    static void reset_response(rmcs_msgs::srv::Relocalize::Response& response) {
        response.success = false;
        response.message = "";
        response.fitness_score = std::numeric_limits<float>::infinity();
        response.within_field_bounds = false;
        response.confidence = 0.0F;
    }

    // 发布当前world->odom坐标变换到TF2
    void publish_current_world_to_odom() {
        auto stamp = geometry_msgs::msg::TransformStamped{};
        {
            auto lock = std::scoped_lock{state_mutex_};
            stamp = current_world_to_odom_;
        }
        stamp.header.stamp = node_.now();
        tf_broadcaster_->sendTransform(stamp);
    }

    // 查询当前odom->base坐标变换
    auto lookup_odom_to_base(Eigen::Isometry3f& transform) const -> bool {
        try {
            const auto stamp =
                tf_buffer_.lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
            transform = tools::transform_to_isometry(stamp.transform);
            return true;
        } catch (const tf2::TransformException& error) {
            RCLCPP_WARN(
                node_.get_logger(), "lookup %s->%s failed: %s", odom_frame_.c_str(),
                base_frame_.c_str(), error.what());
            return false;
        }
    }

    // 从地图中提取子地图
    auto extract_submap(const Eigen::Vector3f& center, double radius_m) const
        -> std::shared_ptr<PointCloud> {
        return tools::extract_submap_radius(map_kdtree_, map_cloud_, center, radius_m, 25.0);
    }

    /**
     * @brief 点云收集结果的封装结构体
     */
    struct CollectResult {
        std::shared_ptr<PointCloud> cloud;
        std::string error_message;
        bool ok = false;
    };

    /**
     * @brief 点云采集与最低点数校验
     *
     * 对 topic 和 duration 应用默认值回退逻辑后，调用 Collector 采集 odom 点云并检查点数是否达标。
     */
    auto collect_and_validate_points(
        const rmcs_msgs::srv::Relocalize::Request& request, const std::string& default_topic,
        double default_duration_sec, int min_points) -> CollectResult {
        auto query_topic = request.pointcloud_topic;
        if (query_topic.empty())
            query_topic = default_topic;

        const auto duration_sec = request.collect_duration_sec <= 0.0f
                                    ? default_duration_sec
                                    : static_cast<double>(request.collect_duration_sec);

        auto query_cloud =
            collector_->collect(node_, tf_buffer_, pointcloud_group_, query_topic, duration_sec);
        if (!query_cloud
            || query_cloud->size() < static_cast<std::size_t>(std::max(1, min_points))) {
            return CollectResult{
                .cloud = nullptr,
                .error_message = "insufficient query cloud points",
                .ok = false,
            };
        }

        return CollectResult{
            .cloud = std::move(query_cloud),
            .error_message = "",
            .ok = true,
        };
    }

    //更新 world->odom 变换
    void update_and_publish_world_to_odom(const Eigen::Isometry3f& world_to_odom) {
        auto new_transform = geometry_msgs::msg::TransformStamped{};
        new_transform.header.frame_id = world_frame_;
        new_transform.child_frame_id = odom_frame_;
        new_transform.transform = tools::isometry_to_transform(world_to_odom);

        {
            auto lock = std::scoped_lock{state_mutex_};
            current_world_to_odom_ = new_transform;
        }
        publish_current_world_to_odom();
    }

    /**
     * @brief 处理初始重定位（MODE_INITIAL）
     *
     * 流程：odom 查询 → 点云采集 → 以初猜位置为中心裁剪地图子图 → 多阶段 ICP 配准 →
     * 结果验收 → 更新 world->odom。
     */
    void handle_initial_relocalize(
        const rmcs_msgs::srv::Relocalize::Request& request,
        rmcs_msgs::srv::Relocalize::Response& response) {
        if (!map_available_) {
            response.message = "map unavailable";
            return;
        }

        // 查询当前odom->base坐标变换
        auto odom_to_base_now = Eigen::Isometry3f::Identity();
        if (!lookup_odom_to_base(odom_to_base_now)) {
            response.message = "failed to query odom->base";
            return;
        }

        //构建初始guess变换
        const auto world_to_base_guess = tools::pose_to_isometry(request.initial_guess_world_base);
        const auto world_to_odom_guess = world_to_base_guess * odom_to_base_now.inverse();

        //收集验证点云数据
        const auto query_result = collect_and_validate_points(
            request, initial_runtime_config_.pointcloud_topic,
            initial_runtime_config_.collect_duration_sec,
            initial_runtime_config_.min_accumulated_points);
        if (!query_result.ok) {
            response.message = query_result.error_message;
            return;
        }

        //提取子地图
        auto map_submap = extract_submap(
            world_to_base_guess.translation(), initial_runtime_config_.submap_radius_m);
        if (!map_submap || map_submap->empty()) {
            response.message = "no map submap around initial guess";
            return;
        }

        // 执行点云配准
        auto world_to_odom_result = Eigen::Isometry3f::Identity();
        double score = std::numeric_limits<double>::infinity();
        if (!tools::run_initial(
                initial_registration_config_, query_result.cloud, map_submap, world_to_odom_guess,
                world_to_odom_result, score)) {
            response.message = "initial registration failed";
            return;
        }

        //查询最终odom->base坐标变换
        auto odom_to_base_final = Eigen::Isometry3f::Identity();
        if (!lookup_odom_to_base(odom_to_base_final)) {
            response.message = "failed to query odom->base after registration";
            return;
        }
        const auto world_to_base_estimated = world_to_odom_result * odom_to_base_final;

        //验证结果
        const auto validation =
            validator_->evaluate_initial(world_to_base_guess, world_to_base_estimated, score);

        response.estimated_world_base = tools::isometry_to_pose(world_to_base_estimated);
        response.world_to_odom = tools::isometry_to_transform(world_to_odom_result);
        response.fitness_score = static_cast<float>(score);
        response.within_field_bounds = validation.within_bounds;

        if (!validation.accepted) {
            const auto estimated_yaw = std::atan2(
                static_cast<double>(world_to_base_estimated.rotation()(1, 0)),
                static_cast<double>(world_to_base_estimated.rotation()(0, 0)));
            const auto guessed_yaw = std::atan2(
                static_cast<double>(world_to_base_guess.rotation()(1, 0)),
                static_cast<double>(world_to_base_guess.rotation()(0, 0)));
            const auto yaw_error_deg = std::abs(
                                           std::atan2(
                                               std::sin(estimated_yaw - guessed_yaw),
                                               std::cos(estimated_yaw - guessed_yaw)))
                * 180.0 / std::numbers::pi;
            const auto distance_error_m =
                (world_to_base_estimated.translation() - world_to_base_guess.translation()).norm();

            response.message =
                build_validation_failure_message("initial", validation, /*include_inlier=*/false);
            RCLCPP_WARN(
                node_.get_logger(),
                "initial rejected: score=%.4f/%.4f, distance=%.3f/%.3f, yaw_deg=%.2f/%.2f",
                score, initial_validation_config_.score_threshold, distance_error_m,
                initial_validation_config_.initial_max_translation_error_m, yaw_error_deg,
                initial_validation_config_.initial_max_yaw_error_deg);
            return;
        }

        update_and_publish_world_to_odom(world_to_odom_result);

        response.success = true;
        response.message = "ok";
    }

    /// 构造 LOCAL/WIDE 共用 prior（去 sigma 后只剩 has_prior + world_to_base + odom_to_base）
    static auto make_prior(
        const rmcs_msgs::srv::Relocalize::Request& request,
        const Eigen::Isometry3f& odom_to_base_before) -> RegistrationPrior {
        auto prior = RegistrationPrior{};
        prior.has_prior = true;
        prior.world_to_base = tools::pose_to_isometry(request.initial_guess_world_base);
        prior.odom_to_base = odom_to_base_before;
        return prior;
    }

    /**
     * @brief MODE_LOCAL：单 seed 围绕 prior 配准，依赖 prior 准确。热路径。
     */
    void handle_local_relocalize(
        const rmcs_msgs::srv::Relocalize::Request& request,
        rmcs_msgs::srv::Relocalize::Response& response) {
        if (!map_available_) {
            response.message = "map unavailable";
            return;
        }

        auto odom_to_base_before = Eigen::Isometry3f::Identity();
        if (!lookup_odom_to_base(odom_to_base_before)) {
            response.message = "failed to query odom->base";
            return;
        }

        const auto query_result = collect_and_validate_points(
            request, local_runtime_config_.pointcloud_topic,
            local_runtime_config_.collect_duration_sec,
            local_runtime_config_.min_accumulated_points);
        if (!query_result.ok) {
            response.message = query_result.error_message;
            return;
        }

        const auto prior = make_prior(request, odom_to_base_before);
        auto registration_result = RegistrationResult{};
        if (!tools::run_local(
                initial_registration_config_, local_registration_config_, query_result.cloud,
                map_cloud_, map_kdtree_, prior, registration_result)) {
            response.message = "local registration failed";
            return;
        }

        auto odom_to_base_after = Eigen::Isometry3f::Identity();
        if (!lookup_odom_to_base(odom_to_base_after)) {
            response.message = "failed to query odom->base after registration";
            return;
        }

        const auto world_to_base_estimated = registration_result.world_to_odom * odom_to_base_after;
        const auto validation = validator_->evaluate_local(
            prior, world_to_base_estimated, registration_result.score,
            registration_result.inlier_ratio);

        response.estimated_world_base = tools::isometry_to_pose(world_to_base_estimated);
        response.world_to_odom = tools::isometry_to_transform(registration_result.world_to_odom);
        response.fitness_score = static_cast<float>(registration_result.score);
        response.within_field_bounds = validation.within_bounds;
        response.confidence = validation.confidence;

        if (!validation.accepted) {
            response.message = build_validation_failure_message("local", validation, true);
            return;
        }

        update_and_publish_world_to_odom(registration_result.world_to_odom);
        response.success = true;
        response.message = "ok";
    }

    /// 把 SC 描述子返回的 (world_position, yaw_deg) 候选转为 world_to_base seed
    static auto sc_match_to_seed(const ScanContextMatch& match) -> Eigen::Isometry3f {
        const auto yaw_radian = static_cast<float>(match.yaw_deg * std::numbers::pi / 180.0);
        auto seed = Eigen::Isometry3f::Identity();
        seed.translation() = match.world_position;
        seed.linear() =
            Eigen::AngleAxisf{ yaw_radian, Eigen::Vector3f::UnitZ() }.toRotationMatrix();
        return seed;
    }

    /// 把 odom 系点云平移到 robot-centered（odom_to_base 的位置变成原点），用于构造 SC 描述子
    static auto translate_to_robot_frame(
        const std::shared_ptr<PointCloud>& query_odom_cloud,
        const Eigen::Isometry3f& odom_to_base) -> std::shared_ptr<PointCloud> {
        auto robot_centered = std::make_shared<PointCloud>();
        robot_centered->reserve(query_odom_cloud->size());
        const auto offset = odom_to_base.translation();
        for (const auto& point : query_odom_cloud->points) {
            Point shifted{};
            shifted.x = point.x - offset.x();
            shifted.y = point.y - offset.y();
            shifted.z = point.z;
            robot_centered->push_back(shifted);
        }
        robot_centered->width = static_cast<std::uint32_t>(robot_centered->size());
        robot_centered->height = 1;
        robot_centered->is_dense = query_odom_cloud->is_dense;
        return robot_centered;
    }

    /**
     * @brief MODE_WIDE：全局兜底。SC 描述子库可用时走 SC top-K 主路径；不可用时走 fallback 多 seed
     */
    void handle_wide_relocalize(
        const rmcs_msgs::srv::Relocalize::Request& request,
        rmcs_msgs::srv::Relocalize::Response& response) {
        if (!map_available_) {
            response.message = "map unavailable";
            return;
        }

        auto odom_to_base_before = Eigen::Isometry3f::Identity();
        if (!lookup_odom_to_base(odom_to_base_before)) {
            response.message = "failed to query odom->base";
            return;
        }

        const auto query_result = collect_and_validate_points(
            request, wide_runtime_config_.pointcloud_topic,
            wide_runtime_config_.collect_duration_sec,
            wide_runtime_config_.min_accumulated_points);
        if (!query_result.ok) {
            response.message = query_result.error_message;
            return;
        }

        const auto prior = make_prior(request, odom_to_base_before);

        // 准备 seed 列表：SC 主路径优先，不可用降级 fallback
        auto seeds = std::vector<Eigen::Isometry3f>{};
        const auto* path_label = "fallback";
        if (scan_context_available_) {
            const auto query_local =
                translate_to_robot_frame(query_result.cloud, odom_to_base_before);
            const auto query_descriptor =
                tools::build_descriptor(*query_local, scan_context_config_);
            const auto matches = map_descriptor_db_.query(
                query_descriptor, wide_registration_config_.sc_top_k);
            if (!matches.empty()) {
                seeds.reserve(matches.size());
                for (const auto& match : matches)
                    seeds.push_back(sc_match_to_seed(match));
                path_label = "scan_context";
                RCLCPP_INFO(
                    node_.get_logger(),
                    "wide: SC returned %zu candidates (top sc_score=%.4f)", matches.size(),
                    matches.front().sc_score);
            } else {
                RCLCPP_WARN(
                    node_.get_logger(), "wide: SC query empty, falling back to multi-seed");
            }
        }
        if (seeds.empty()) {
            seeds = tools::build_wide_fallback_seeds(prior, wide_registration_config_);
            RCLCPP_INFO(
                node_.get_logger(), "wide: using %zu fallback seeds (path=%s)", seeds.size(),
                path_label);
        }

        auto registration_result = RegistrationResult{};
        if (!tools::run_wide(
                initial_registration_config_, wide_registration_config_, query_result.cloud,
                map_cloud_, map_kdtree_, prior, seeds, registration_result)) {
            response.message =
                std::string{"wide registration failed (path="} + path_label + ")";
            return;
        }

        auto odom_to_base_after = Eigen::Isometry3f::Identity();
        if (!lookup_odom_to_base(odom_to_base_after)) {
            response.message = "failed to query odom->base after registration";
            return;
        }

        const auto world_to_base_estimated = registration_result.world_to_odom * odom_to_base_after;
        const auto validation = validator_->evaluate_wide(
            prior, world_to_base_estimated, registration_result.score,
            registration_result.inlier_ratio);

        response.estimated_world_base = tools::isometry_to_pose(world_to_base_estimated);
        response.world_to_odom = tools::isometry_to_transform(registration_result.world_to_odom);
        response.fitness_score = static_cast<float>(registration_result.score);
        response.within_field_bounds = validation.within_bounds;
        response.confidence = validation.confidence;

        if (!validation.accepted) {
            response.message = build_validation_failure_message("wide", validation, true);
            return;
        }

        update_and_publish_world_to_odom(registration_result.world_to_odom);
        response.success = true;
        response.message = "ok";
    }

    void handle_relocalize(
        const std::shared_ptr<rmcs_msgs::srv::Relocalize::Request> request,
        std::shared_ptr<rmcs_msgs::srv::Relocalize::Response> response) {
        if (!response)
            return;

        reset_response(*response);
        if (!request) {
            response->message = "invalid request";
            return;
        }

        {
            auto lock = std::scoped_lock{state_mutex_};
            if (busy_) {
                response->message = "busy";
                return;
            }
            busy_ = true;
        }
        auto busy_guard = BusyGuard{*this};

        std::string_view mode_label = "unknown";
        switch (request->mode) {
        case rmcs_msgs::srv::Relocalize::Request::MODE_INITIAL:
            mode_label = "initial";
            handle_initial_relocalize(*request, *response);
            break;
        case rmcs_msgs::srv::Relocalize::Request::MODE_LOCAL:
            mode_label = "local";
            handle_local_relocalize(*request, *response);
            break;
        case rmcs_msgs::srv::Relocalize::Request::MODE_WIDE:
            mode_label = "wide";
            handle_wide_relocalize(*request, *response);
            break;
        default: response->message = "unsupported relocalization mode"; break;
        }
        log_if_failed(mode_label, *request, *response);
    }

    /// 拼接 "<mode> registration rejected: tag1,tag2,..." —— INITIAL/LOCAL/WIDE 共用
    static auto build_validation_failure_message(
        std::string_view mode, const ValidationResult& validation, bool include_inlier)
        -> std::string {
        const auto add = [&](std::string& msg, bool failed, std::string_view tag, bool& first) {
            if (!failed) return;
            msg += first ? ": " : ",";
            msg += tag;
            first = false;
        };
        auto msg = std::string{mode} + " registration rejected";
        auto first = true;
        add(msg, !validation.within_bounds, "out_of_bounds", first);
        add(msg, !validation.score_ok, "score", first);
        if (include_inlier)
            add(msg, !validation.inlier_ok, "inlier", first);
        add(msg, !validation.distance_ok, "distance", first);
        add(msg, !validation.yaw_ok, "yaw", first);
        return msg;
    }

    /// 失败时打一行带上下文的 warn（mode + msg + prior 位置），由 dispatcher 在 handler 返回后调用
    void log_if_failed(
        std::string_view mode, const rmcs_msgs::srv::Relocalize::Request& request,
        const rmcs_msgs::srv::Relocalize::Response& response) const {
        if (!log_failure_details_ || response.success)
            return;
        const auto& p = request.initial_guess_world_base.position;
        RCLCPP_WARN(
            node_.get_logger(),
            "relocalize_failed mode=%.*s prior=(%.2f,%.2f,%.2f) score=%.4f conf=%.3f | %s",
            static_cast<int>(mode.size()), mode.data(), p.x, p.y, p.z,
            static_cast<double>(response.fitness_score),
            static_cast<double>(response.confidence), response.message.c_str());
    }

    RelocalizationServer& node_;

    std::string map_path_;
    std::string descriptor_path_;
    std::string service_name_;
    std::string world_frame_;
    std::string odom_frame_;
    std::string base_frame_;

    double publish_tf_rate_hz_ = 10.0;

    InitialRuntimeConfig initial_runtime_config_{};
    LocalRuntimeConfig local_runtime_config_{};
    WideRuntimeConfig wide_runtime_config_{};

    InitialRegistrationConfig initial_registration_config_{};
    LocalRegistrationConfig local_registration_config_{};
    WideRegistrationConfig wide_registration_config_{};

    InitialValidationConfig initial_validation_config_{};
    LocalValidationConfig local_validation_config_{};
    WideValidationConfig wide_validation_config_{};

    ScanContextConfig scan_context_config_{};
    MapDescriptorDB map_descriptor_db_{};
    bool scan_context_available_ = false;

    bool log_failure_details_ = false;

    bool map_available_ = false;
    std::shared_ptr<PointCloud> map_cloud_;
    pcl::KdTreeFLANN<Point> map_kdtree_;
    bool map_kdtree_ready_ = false;

    std::unique_ptr<Collector> collector_;
    std::unique_ptr<Validator> validator_;

    rclcpp::CallbackGroup::SharedPtr service_group_;
    rclcpp::CallbackGroup::SharedPtr pointcloud_group_;
    rclcpp::Service<rmcs_msgs::srv::Relocalize>::SharedPtr relocalize_service_;

    std::shared_ptr<rclcpp::TimerBase> tf_publish_timer_;

    mutable tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    mutable std::mutex state_mutex_;
    bool busy_ = false;
    geometry_msgs::msg::TransformStamped current_world_to_odom_;
};

RelocalizationServer::RelocalizationServer()
    : Node("rmcs_relocation", rclcpp::NodeOptions{})
    , pimpl_(std::make_unique<Impl>(*this)) {}

RelocalizationServer::~RelocalizationServer() = default;

} // namespace rmcs::location

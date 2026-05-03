/**
 * @file runtime.cpp
 * @brief 重定位服务器运行实现
 *
 * 该文件实现了重定位服务器的主要逻辑，包括参数加载、服务处理、
 * 点云收集、配准算法执行和坐标变换发布等功能。
 * 支持初始重定位和丢失重定位两种模式。
 *
 * @author RMCS Development Team
 */

#include "runtime.hpp"

#include "collector.hpp"
#include "tools/geometry_tools.hpp"
#include "tools/numeric_tools.hpp"
#include "tools/param_tools.hpp"
#include "tools/registration_tools.hpp"
#include "validator.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
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

/**
 * @brief 重定位服务器内部实现
 *
 * 承载完整的重定位服务生命周期：加载地图与参数、响应 INITIAL/LOST 重定位请求、
 * 维护 world->odom 变换发布。
 */
struct RelocalizationServer::Impl {
    /**
     * @brief 忙等保护 RAII 守卫
     *
     * 在 handle_relocalize 中标记 busy，析构时自动清除。
     * 确保同一时刻只有一条重定位请求在处理。
     */
    struct BusyGuard {
        explicit BusyGuard(Impl& impl)
            : impl_(impl) {}

        ~BusyGuard() {
            auto lock = std::scoped_lock{impl_.state_mutex_};
            impl_.busy_ = false;
        }

        Impl& impl_;
    };

    /**
     * @brief 构造服务器内部实现
     *
     * 执行初始化序列：加载参数 → 初始化子模块 → 初始化 TF 变换缓存 → 加载地图 →
     * 创建回调组 → 创建服务 → 启动 TF 定时发布。
     */
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
        service_name_ = std::move(params.service_name);
        world_frame_ = std::move(params.world_frame);
        odom_frame_ = std::move(params.odom_frame);
        base_frame_ = std::move(params.base_frame);
        publish_tf_rate_hz_ = params.publish_tf_rate_hz;

        initial_runtime_config_ = std::move(params.initial_runtime_config);
        lost_runtime_config_ = std::move(params.lost_runtime_config);

        initial_registration_config_ = std::move(params.initial_registration_config);
        lost_registration_config_ = std::move(params.lost_registration_config);

        initial_validation_config_ = std::move(params.initial_validation_config);
        lost_validation_config_ = std::move(params.lost_validation_config);
    }

    //初始化子模块：点云收集器、验证器
    void initialize_modules() {
        collector_ = std::make_unique<Collector>(odom_frame_);
        validator_ =
            std::make_unique<Validator>(initial_validation_config_, lost_validation_config_);
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

    /**
     * @brief 加载 PCD 地图文件并构建 KD 树
     *
     * 支持空路径（无地图模式），此时 map_available_ = false。
     */
    void load_map() {
        map_cloud_ = std::make_shared<PointCloud>();
        if (map_path_.empty()) {
            map_available_ = false;
            RCLCPP_WARN(
                node_.get_logger(), "map_path is empty, relocalization will be unavailable\n");
            return;
        }

        if (pcl::io::loadPCDFile(map_path_, *map_cloud_) != 0) {
            map_available_ = false;
            RCLCPP_ERROR(node_.get_logger(), "failed to load map from %s\n", map_path_.c_str());
            return;
        }

        //地图转换 - 下移0.56米，让地图原点对齐到底盘中心
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation().z() = 0.56f;  // 下移0.56米
        auto transformed_cloud = std::make_shared<PointCloud>();
        pcl::transformPointCloud(*map_cloud_, *transformed_cloud, transform);
        map_cloud_ = transformed_cloud;

        map_kdtree_.setInputCloud(map_cloud_);
        map_kdtree_ready_ = true;
        map_available_ = true;

        RCLCPP_INFO(
            node_.get_logger(), "map loaded: %s (%zu points)\n", map_path_.c_str(),
            map_cloud_->size());
    }

    // 在开始处理重定位请求前，将响应对象重置为默认的失败状态
    static void reset_response(rmcs_msgs::srv::Relocalize::Response& response) {
        response.success = false;
        response.message = "";
        response.fitness_score = std::numeric_limits<float>::infinity();
        response.within_field_bounds = false;
        response.confidence = 0.0F;
        response.tier_used = 255;
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
                node_.get_logger(), "lookup %s->%s failed: %s\n", odom_frame_.c_str(),
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

    /**
     * @brief 更新缓存的 world->odom 变换并立即广播
     *
     * 在配准成功后的最后一步调用，使外部 TF 消费者能立即看到新结果。
     */
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
            response.message = "initial registration result rejected by acceptance checks";
            return;
        }

        update_and_publish_world_to_odom(world_to_odom_result);

        response.success = true;
        response.message = "ok";
    }

    /**
     * @brief 处理丢失重定位（MODE_LOST）
     *
     * 流程：odom 查询 → 点云采集 → 构建先验信息 → 多候选种子精配（按 sigma 切 LOCAL/WIDE 档） →
     * 结果验收 → 更新 world->odom。验收通过后返回 tier_used 和 confidence。
     */
    void handle_lost_relocalize(
        const rmcs_msgs::srv::Relocalize::Request& request,
        rmcs_msgs::srv::Relocalize::Response& response) {
        if (!map_available_) {
            response.message = "map unavailable";
            return;
        }

        auto odom_to_base_before = Eigen::Isometry3f::Identity();
        if (!lookup_odom_to_base(odom_to_base_before)) {
            response.message = "failed to query odom->base\n";
            return;
        }

        const auto query_result = collect_and_validate_points(
            request, lost_runtime_config_.pointcloud_topic,
            lost_runtime_config_.collect_duration_sec, lost_runtime_config_.min_accumulated_points);
        if (!query_result.ok) {
            response.message = query_result.error_message;
            return;
        }

        // 构建先验信息，加入sigma值作为不确定参数
        //sigma值的使用将在配准算法中以加权方式影响种子生成和优化过程，避免因先验信息不准确导致搜索失败
        auto prior = LostPrior{};
        prior.has_prior = true;
        prior.world_to_base = tools::pose_to_isometry(request.initial_guess_world_base);
        prior.odom_to_base = odom_to_base_before;
        prior.sigma_xy_m = request.prior_sigma_xy_m > 0.0F
                             ? static_cast<double>(request.prior_sigma_xy_m)
                             : lost_registration_config_.local_sigma_xy_m + 1.0;
        prior.sigma_yaw_deg = request.prior_sigma_yaw_deg > 0.0F
                                ? static_cast<double>(request.prior_sigma_yaw_deg)
                                : lost_registration_config_.local_sigma_yaw_deg + 1.0;

        //点云配准
        auto lost_result = LostResult{};
        if (!tools::run_lost(
                initial_registration_config_, lost_registration_config_, query_result.cloud,
                map_cloud_, map_kdtree_, prior, lost_result)) {
            response.message = "lost registration failed";
            return;
        }

        auto odom_to_base_after = Eigen::Isometry3f::Identity();
        if (!lookup_odom_to_base(odom_to_base_after)) {
            response.message = "failed to query odom->base after registration";
            return;
        }

        const auto world_to_base_estimated = lost_result.world_to_odom * odom_to_base_after;
        const auto validation = validator_->evaluate_lost(
            prior, world_to_base_estimated, lost_result.score, lost_result.inlier_ratio,
            lost_result.tier_used);

        response.estimated_world_base = tools::isometry_to_pose(world_to_base_estimated);
        response.world_to_odom = tools::isometry_to_transform(lost_result.world_to_odom);
        response.fitness_score = static_cast<float>(lost_result.score);
        response.within_field_bounds = validation.within_bounds;
        response.confidence = validation.confidence;
        response.tier_used = static_cast<std::uint8_t>(lost_result.tier_used);

        if (!validation.accepted) {
            response.message = build_validation_failure_message(validation);
            return;
        }

        update_and_publish_world_to_odom(lost_result.world_to_odom);

        response.success = true;
        response.message = "ok";
    }

    /**
     * @brief 处理重定位服务请求
     *
     * 服务处理流程：
     * 使用RAII模式管理忙碌状态
     *
     * @param request 重定位服务请求
     * @param response 重定位服务响应
     */
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

        switch (request->mode) {
        case rmcs_msgs::srv::Relocalize::Request::MODE_INITIAL:
            handle_initial_relocalize(*request, *response);
            break;
        case rmcs_msgs::srv::Relocalize::Request::MODE_LOST:
            handle_lost_relocalize(*request, *response);
            break;
        case rmcs_msgs::srv::Relocalize::Request::MODE_MANUAL:
            RCLCPP_WARN(
                node_.get_logger(),
                "MODE_MANUAL is deprecated and currently routed to MODE_INITIAL behavior\n");
            handle_initial_relocalize(*request, *response);
            break;
        default: response->message = "unsupported relocalization mode"; break;
        }
    }

    /**
     * @brief 将验证结果中的失败项拼接打印
     */
    static auto build_validation_failure_message(const ValidationResult& validation)
        -> std::string {
        const auto checks = std::array<std::pair<bool, std::string_view>, 5>{{
            {!validation.within_bounds, "out_of_bounds"},
            {!validation.score_ok, "score"},
            {!validation.inlier_ok, "inlier"},
            {!validation.distance_ok, "distance"},
            {!validation.yaw_ok, "yaw"},
        }};

        auto message = std::string{"lost registration rejected"};
        auto first_reason = true;
        for (const auto& [failed, reason] : checks) {
            if (!failed)
                continue;
            message += first_reason ? ": " : ",";
            message += reason;
            first_reason = false;
        }
        return message;
    }

    RelocalizationServer& node_;

    std::string map_path_;
    std::string service_name_;
    std::string world_frame_;
    std::string odom_frame_;
    std::string base_frame_;

    double publish_tf_rate_hz_ = 10.0;

    InitialRuntimeConfig initial_runtime_config_{};
    LostRuntimeConfig lost_runtime_config_{};

    InitialRegistrationConfig initial_registration_config_{};
    LostRegistrationConfig lost_registration_config_{};

    InitialValidationConfig initial_validation_config_{};
    LostValidationConfig lost_validation_config_{};

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

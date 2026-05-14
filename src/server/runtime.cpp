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
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <rmcs_relocation/srv/relocalize.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace rmcs::location {

namespace {

/// 把 SC 描述子返回的 (world_position, yaw_deg) 候选转为 world_to_base seed。
///
/// yaw 推导：
///   - 库描述子在 world 系朝向构造（grid 中心仅 XY 平移，不旋转）
///   - query 描述子在 odom 系朝向构造（translate_to_robot_frame 仅 XY 平移，不旋转）
///   - 因此 SC 列位移找到的 yaw（match.yaw_deg）= world_to_odom 的 yaw（α）
///   - 真实 world_to_base.yaw = α + odom_to_base.yaw（β），β 来自 LIO
///   - seed.translation 是 grid 中心的 world 坐标
auto sc_match_to_seed(const ScanContextMatch& match, const Eigen::Isometry3f& odom_to_base)
    -> Eigen::Isometry3f {
    const auto sc_yaw_radian = static_cast<float>(match.yaw_deg * std::numbers::pi / 180.0);
    const auto base_yaw_radian = static_cast<float>(std::atan2(
        static_cast<double>(odom_to_base.rotation()(1, 0)),
        static_cast<double>(odom_to_base.rotation()(0, 0))));
    const auto seed_yaw_radian = sc_yaw_radian + base_yaw_radian;

    auto seed = Eigen::Isometry3f::Identity();
    seed.translation() = match.world_position;
    seed.linear() = Eigen::AngleAxisf{seed_yaw_radian, Eigen::Vector3f::UnitZ()}.toRotationMatrix();
    return seed;
}

/// 把 odom 系点云平移到机器人当前位置（仅平移，不做旋转），用于构造 SC 描述子
auto translate_to_robot_frame(
    const std::shared_ptr<PointCloud>& query_odom_cloud, const Eigen::Isometry3f& odom_to_base)
    -> std::shared_ptr<PointCloud> {
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

auto build_validation_failure_message(
    std::string_view mode, const ValidationResult& validation, bool include_inlier) -> std::string {
    const auto add = [](std::string& msg, bool failed, std::string_view tag, bool& first) {
        if (!failed)
            return;
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

auto extract_yaw_deg(const Eigen::Isometry3f& pose) -> double {
    return std::atan2(
               static_cast<double>(pose.rotation()(1, 0)),
               static_cast<double>(pose.rotation()(0, 0)))
         * 180.0 / std::numbers::pi;
}

} // namespace

struct RelocalizationServer::Impl {
    // 在 handle_relocalize 中标记 busy，析构时自动清除。确保同一时刻只有一条重定位请求在处理。
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

        relocalize_service_ = node_.create_service<rmcs_relocation::srv::Relocalize>(
            service_name_,
            [this](
                const std::shared_ptr<rmcs_relocation::srv::Relocalize::Request> request,
                std::shared_ptr<rmcs_relocation::srv::Relocalize::Response> response) {
                handle_relocalize(request, std::move(response));
            },
            rclcpp::ServicesQoS(), service_group_);

        const auto publish_interval = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / std::max(1.0, publish_tf_rate_hz_)));
        tf_publish_timer_ =
            node_.create_wall_timer(publish_interval, [this] { publish_current_world_to_odom(); });
    }

    // 从 ROS 参数服务器一次性加载全部运行时参数
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
        local_safety_config_ = std::move(params.local_safety_config);
        wide_runtime_config_ = std::move(params.wide_runtime_config);

        common_registration_config_ = std::move(params.common_registration_config);
        initial_registration_config_ = std::move(params.initial_registration_config);
        local_registration_config_ = std::move(params.local_registration_config);
        wide_registration_config_ = std::move(params.wide_registration_config);

        initial_validation_config_ = std::move(params.initial_validation_config);
        local_validation_config_ = std::move(params.local_validation_config);
        wide_validation_config_ = std::move(params.wide_validation_config);

        scan_context_config_ = params.scan_context_config;
        log_failure_details_ = params.log_failure_details;
    }

    // 初始化子模块：点云收集器、验证器
    void initialize_modules() {
        collector_ = std::make_unique<Collector>();
        validator_ = std::make_unique<Validator>(
            initial_validation_config_, local_validation_config_, wide_validation_config_);
    }

    void initialize_transform_cache() {
        auto initial = geometry_msgs::msg::TransformStamped{};
        initial.header.frame_id = world_frame_;
        initial.child_frame_id = odom_frame_;
        initial.transform.rotation.w = 1.0;

        auto lock = std::scoped_lock{state_mutex_};
        current_world_to_odom_ = initial;
    }

    // 加载 PCD 地图文件并构建 KD 树；若提供 descriptor_path 则尝试加载 SC 描述子库
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

        // 启动时一次性 voxel + outlier 全图；所有 GICP 路径（initial/local/wide）
        // 共享这份 target，不再按 seed 切 submap。
        map_voxelized_cloud_ =
            tools::preprocess_map(map_cloud_, common_registration_config_.preprocess);
        if (!map_voxelized_cloud_ || map_voxelized_cloud_->empty()) {
            map_available_ = false;
            RCLCPP_ERROR(
                node_.get_logger(),
                "map preprocess produced empty cloud (raw=%zu, voxel_leaf=%.3f); relocalization "
                "will be unavailable",
                map_cloud_->size(), common_registration_config_.preprocess.voxel_leaf_m);
            return;
        }

        map_available_ = true;

        RCLCPP_INFO(
            node_.get_logger(), "map loaded: %s (raw=%zu, voxel_leaf=%.3f, after_preprocess=%zu)",
            map_path_.c_str(), map_cloud_->size(),
            common_registration_config_.preprocess.voxel_leaf_m, map_voxelized_cloud_->size());

        load_scan_context_db();
    }

    /// 加载 SC 描述子库（.sc_desc）。失败则关闭 SC，wide 直接失败并等待决策端重试。
    void load_scan_context_db() {
        scan_context_available_ = false;
        if (descriptor_path_.empty()) {
            RCLCPP_INFO(node_.get_logger(), "descriptor_path is empty, scan_context disabled");
            return;
        }

        const auto map_hash = tools::compute_map_hash(*map_cloud_);
        if (!map_descriptor_db_.load(descriptor_path_, scan_context_config_, map_hash)) {
            RCLCPP_WARN(
                node_.get_logger(),
                "scan_context descriptor load failed (path=%s, expected_hash=0x%08x)",
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
    static void reset_response(rmcs_relocation::srv::Relocalize::Response& response) {
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

    // 查询当前 odom->base 坐标变换（最新可用）
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

    /// 按指定 ROS 时刻查询 odom->base，允许 timeout 秒等待 TF 到齐。
    /// 失败不打印 WARN（调用方按业务语义决定 log 等级）。
    auto lookup_odom_to_base_at(
        const rclcpp::Time& stamp, double timeout_sec, Eigen::Isometry3f& transform,
        std::string& error_message) const -> bool {
        try {
            const auto timeout = rclcpp::Duration::from_seconds(std::max(0.0, timeout_sec));
            const auto tf_msg =
                tf_buffer_.lookupTransform(odom_frame_, base_frame_, stamp, timeout);
            transform = tools::transform_to_isometry(tf_msg.transform);
            return true;
        } catch (const tf2::TransformException& error) {
            error_message = error.what();
            return false;
        }
    }

    /**
     * @brief 点云收集结果的封装结构体
     *
     * reference_stamp 是本次累积点云首末帧 stamp 的中点，用于按时间查询 odom->base
     * 作为 SC / GICP 的统一先验锚点（snapshot 参考时刻对齐）。
     */
    struct CollectResult {
        std::shared_ptr<PointCloud> cloud;
        rclcpp::Time reference_stamp{0, 0, RCL_ROS_TIME};
        std::size_t frame_count = 0;
        std::string error_message;
        bool ok = false;
    };

    /**
     * @brief 点云采集与最低点数校验
     *
     * 对 topic 和 duration 应用默认值回退逻辑后，调用 Collector 采集 odom 点云并检查点数是否达标。
     * 输入话题必须发布 odom 系点云（frame_id == odom_frame_），不匹配的帧会被 Collector 丢弃。
     */
    auto collect_and_validate_points(
        const rmcs_relocation::srv::Relocalize::Request& request, const std::string& default_topic,
        double default_duration_sec, int min_points) -> CollectResult {
        auto query_topic = request.pointcloud_topic;
        if (query_topic.empty())
            query_topic = default_topic;

        const auto duration_sec = request.collect_duration_sec <= 0.0f
                                    ? default_duration_sec
                                    : static_cast<double>(request.collect_duration_sec);

        // min_points 同时作为 collector 早停阈值：达到即立刻返回，duration 仅做上界。
        auto collected = collector_->collect(
            node_, pointcloud_group_, query_topic, odom_frame_, duration_sec, min_points);
        if (!collected.cloud || collected.frame_count == 0
            || collected.cloud->size() < static_cast<std::size_t>(std::max(1, min_points))) {
            return CollectResult{
                .cloud = nullptr,
                .reference_stamp = rclcpp::Time{0, 0, RCL_ROS_TIME},
                .frame_count = collected.frame_count,
                .error_message = "insufficient query cloud points",
                .ok = false,
            };
        }

        return CollectResult{
            .cloud = std::move(collected.cloud),
            .reference_stamp = collected.reference_stamp,
            .frame_count = collected.frame_count,
            .error_message = "",
            .ok = true,
        };
    }

    /// 把 query 点云搬到机器人本体系 → 构 SC 描述子 → 在描述子库做 top-K 旋转不变匹配。
    /// SC 不可用 / 描述子库返回空时返回空 vector。local 与 wide 共用。
    auto query_sc_matches(
        const std::shared_ptr<PointCloud>& query_odom_cloud, const Eigen::Isometry3f& odom_to_base,
        std::size_t top_k) const -> std::vector<ScanContextMatch> {
        if (!scan_context_available_)
            return {};
        const auto query_local = translate_to_robot_frame(query_odom_cloud, odom_to_base);
        const auto query_descriptor = tools::build_descriptor(*query_local, scan_context_config_);
        return map_descriptor_db_.query(query_descriptor, top_k);
    }

    // 更新 world->odom 变换
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
        const rmcs_relocation::srv::Relocalize::Request& request,
        rmcs_relocation::srv::Relocalize::Response& response) {
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

        // 构建初始guess变换
        const auto world_to_base_guess = tools::pose_to_isometry(request.initial_guess_world_base);
        const auto world_to_odom_guess = world_to_base_guess * odom_to_base_now.inverse();

        // 收集验证点云数据
        const auto query_result = collect_and_validate_points(
            request, initial_runtime_config_.pointcloud_topic,
            initial_runtime_config_.collect_duration_sec,
            initial_runtime_config_.min_accumulated_points);
        if (!query_result.ok) {
            response.message = query_result.error_message;
            return;
        }

        // 执行点云配准（target = 启动时预处理的全图，不再切 submap）
        auto world_to_odom_result = Eigen::Isometry3f::Identity();
        double score = std::numeric_limits<double>::infinity();
        if (!tools::run_initial(
                common_registration_config_, initial_registration_config_, query_result.cloud,
                map_voxelized_cloud_, world_to_odom_guess, world_to_odom_result, score)) {
            response.message = "initial registration failed";
            return;
        }

        // 查询最终odom->base坐标变换
        auto odom_to_base_final = Eigen::Isometry3f::Identity();
        if (!lookup_odom_to_base(odom_to_base_final)) {
            response.message = "failed to query odom->base after registration";
            return;
        }
        const auto world_to_base_estimated = world_to_odom_result * odom_to_base_final;

        // 验证结果
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
                "initial rejected: score=%.4f/%.4f, distance=%.3f/%.3f, yaw_deg=%.2f/%.2f", score,
                initial_validation_config_.score_threshold, distance_error_m,
                initial_validation_config_.initial_max_translation_error_m, yaw_error_deg,
                initial_validation_config_.initial_max_yaw_error_deg);
            return;
        }

        update_and_publish_world_to_odom(world_to_odom_result);

        response.success = true;
        response.message = "ok";
    }

    /// 一次 local seed 尝试的结果：accepted=填响应并 return；rejected=记 message 进
    /// continue；abort=致命错（lookup tf 失败）。
    enum class LocalSeedOutcome { Accepted, Rejected, Abort };

    /// candidate world->odom 相对当前已发布 world->odom 的位姿增量。
    struct TfCorrectionDelta {
        double translation_m;
        double yaw_deg;
    };

    auto compute_tf_correction_delta(const Eigen::Isometry3f& candidate_world_to_odom) const
        -> TfCorrectionDelta {
        auto current = Eigen::Isometry3f::Identity();
        {
            auto lock = std::scoped_lock{state_mutex_};
            current = tools::transform_to_isometry(current_world_to_odom_.transform);
        }
        const auto delta = current.inverse() * candidate_world_to_odom;
        const auto yaw_rad = std::atan2(
            static_cast<double>(delta.rotation()(1, 0)),
            static_cast<double>(delta.rotation()(0, 0)));
        return TfCorrectionDelta{
            .translation_m = static_cast<double>(delta.translation().norm()),
            .yaw_deg = std::abs(std::atan2(std::sin(yaw_rad), std::cos(yaw_rad))) * 180.0
                     / std::numbers::pi,
        };
    }

    /// 导航安全门控：检查单次修正幅度 + 发布速率限制。
    /// 通过返回 true；不通过 reason 写入拒绝原因，调用方填到 response.message。
    auto check_local_safety_gate(const TfCorrectionDelta& delta, std::string& reason) -> bool {
        if (delta.translation_m > local_safety_config_.max_tf_correction_m) {
            reason = "tf_correction_translation (" + std::to_string(delta.translation_m) + "m > "
                   + std::to_string(local_safety_config_.max_tf_correction_m) + "m)";
            return false;
        }
        if (delta.yaw_deg > local_safety_config_.max_tf_correction_yaw_deg) {
            reason = "tf_correction_yaw (" + std::to_string(delta.yaw_deg) + "deg > "
                   + std::to_string(local_safety_config_.max_tf_correction_yaw_deg) + "deg)";
            return false;
        }
        if (last_local_accept_time_.nanoseconds() > 0) {
            const auto elapsed = (node_.now() - last_local_accept_time_).seconds();
            if (elapsed < local_safety_config_.min_accept_interval_sec) {
                reason = "accept_rate (elapsed=" + std::to_string(elapsed) + "s < min_interval="
                       + std::to_string(local_safety_config_.min_accept_interval_sec) + "s)";
                return false;
            }
        }
        return true;
    }

    /// 跑一个 SC seed 的完整 GICP + validator 流程。accepted 时填好 response 并 publish。
    /// rejected 时 last_msg 写入失败原因（GICP 不收敛 / validator 拒收 / TF 门控拒绝）。
    /// abort 表示 lookup_odom_to_base 后失败，调用方应直接结束。
    auto try_local_seed(
        std::size_t seed_index, const ScanContextMatch& match,
        const std::shared_ptr<PointCloud>& query_cloud,
        const Eigen::Isometry3f& odom_to_base_ref, const Eigen::Isometry3f& user_world_to_base,
        rmcs_relocation::srv::Relocalize::Response& response, std::string& last_msg)
        -> LocalSeedOutcome {
        const auto sc_seed = sc_match_to_seed(match, odom_to_base_ref);
        RCLCPP_INFO(
            node_.get_logger(), "local: try seed[%zu] sc_score=%.4f pos=(%.2f,%.2f) yaw=%.1fdeg",
            seed_index, match.sc_score, sc_seed.translation().x(), sc_seed.translation().y(),
            match.yaw_deg);

        auto seed_prior = RegistrationPrior{
            .has_prior = true, .world_to_base = sc_seed, .odom_to_base = odom_to_base_ref};

        auto registration_result = RegistrationResult{};
        if (!tools::run_local(
                common_registration_config_, local_registration_config_, query_cloud,
                map_voxelized_cloud_, seed_prior, registration_result)) {
            last_msg = "local seed[" + std::to_string(seed_index) + "] GICP not converged";
            return LocalSeedOutcome::Rejected;
        }

        auto odom_to_base_now = Eigen::Isometry3f::Identity();
        if (!lookup_odom_to_base(odom_to_base_now)) {
            response.message = "failed to query odom->base after registration";
            return LocalSeedOutcome::Abort;
        }

        const auto world_to_base_estimated = registration_result.world_to_odom * odom_to_base_now;

        // validator 用 user prior 作参考（拦跨半场错配）。
        // 与 GICP/SC 阶段不同，这里 odom_to_base 用 now 而非 ref：validator 比较的是
        // estimated 与 prior 在 world 系下的距离，本身不依赖 odom_to_base 字段。
        const auto validator_prior = RegistrationPrior{
            .has_prior = true,
            .world_to_base = user_world_to_base,
            .odom_to_base = odom_to_base_now};

        const auto validation = validator_->evaluate_local(
            validator_prior, world_to_base_estimated, registration_result.score,
            registration_result.inlier_ratio);

        if (!validation.accepted) {
            last_msg = build_validation_failure_message(
                "local seed[" + std::to_string(seed_index) + "]", validation, true);
            RCLCPP_WARN(
                node_.get_logger(), "local: seed[%zu] rejected g_score=%.4f inlier=%.3f | %s",
                seed_index, registration_result.score, registration_result.inlier_ratio,
                last_msg.c_str());
            return LocalSeedOutcome::Rejected;
        }

        // 导航安全门控：candidate 相对当前发布的 world->odom 增量必须足够小，
        // 且与上次 accept 间隔不能过短。这是 local 不破坏导航连续性的关键。
        const auto delta = compute_tf_correction_delta(registration_result.world_to_odom);
        auto safety_reason = std::string{};
        if (!check_local_safety_gate(delta, safety_reason)) {
            last_msg = "local seed[" + std::to_string(seed_index) + "] rejected: " + safety_reason;
            RCLCPP_WARN(node_.get_logger(), "local: seed[%zu] %s", seed_index, last_msg.c_str());
            // 即便被门控拒绝也回填诊断字段，便于上层判断"为什么没修正"。
            response.estimated_world_base = tools::isometry_to_pose(world_to_base_estimated);
            response.world_to_odom = tools::isometry_to_transform(registration_result.world_to_odom);
            response.fitness_score = static_cast<float>(registration_result.score);
            response.within_field_bounds = validation.within_bounds;
            response.confidence = validation.confidence;
            return LocalSeedOutcome::Rejected;
        }

        const auto estimated_yaw_deg = extract_yaw_deg(world_to_base_estimated);
        const auto prior_yaw_deg = extract_yaw_deg(user_world_to_base);
        const auto distance_error_m =
            (world_to_base_estimated.translation() - user_world_to_base.translation()).norm();
        const auto yaw_error_deg =
            std::abs(
                std::atan2(
                    std::sin((estimated_yaw_deg - prior_yaw_deg) * std::numbers::pi / 180.0),
                    std::cos((estimated_yaw_deg - prior_yaw_deg) * std::numbers::pi / 180.0)))
            * 180.0 / std::numbers::pi;
        // 调参诊断日志：记录 local 验收通过后的最终估计位姿，区别于 SC seed 网格中心。
        RCLCPP_INFO(
            node_.get_logger(),
            "local: seed[%zu] accepted estimated=(%.2f,%.2f,%.1fdeg) "
            "prior=(%.2f,%.2f,%.1fdeg) score=%.4f inlier=%.3f dist=%.3fm yaw_err=%.1fdeg "
            "tf_dx=%.3fm tf_dyaw=%.2fdeg",
            seed_index, world_to_base_estimated.translation().x(),
            world_to_base_estimated.translation().y(), estimated_yaw_deg,
            user_world_to_base.translation().x(), user_world_to_base.translation().y(),
            prior_yaw_deg, registration_result.score, registration_result.inlier_ratio,
            distance_error_m, yaw_error_deg, delta.translation_m, delta.yaw_deg);

        response.estimated_world_base = tools::isometry_to_pose(world_to_base_estimated);
        response.world_to_odom = tools::isometry_to_transform(registration_result.world_to_odom);
        response.fitness_score = static_cast<float>(registration_result.score);
        response.within_field_bounds = validation.within_bounds;
        response.confidence = validation.confidence;
        update_and_publish_world_to_odom(registration_result.world_to_odom);
        last_local_accept_time_ = node_.now();
        response.success = true;
        response.message = "ok (seed " + std::to_string(seed_index) + ")";
        return LocalSeedOutcome::Accepted;
    }

    /**
     * @brief MODE_LOCAL：SC top-K 选种子 + 早停。prior 仅作 validator 安全网（拦镜像错配）。
     *
     * - seed_prior.world_to_base 来自 SC 候选（不依赖 prior 准）
     * - validator_prior.world_to_base 来自 request.initial_guess_world_base（拦跨半场错配）
     * - 早停：seed[0] 通过 validator 立即返回，seed[1] 仅当 seed[0] 失败时才跑
     */
    void handle_local_relocalize(
        const rmcs_relocation::srv::Relocalize::Request& request,
        rmcs_relocation::srv::Relocalize::Response& response) {
        if (!map_available_) {
            response.message = "map unavailable";
            return;
        }
        if (!scan_context_available_) {
            response.message = "SC unavailable";
            return;
        }

        // 先采集点云，得到 reference_stamp（首末帧 stamp 的中点）后再查 odom->base。
        // 这样 SC 描述子查询、SC seed、GICP prior 三者使用同一参考时刻的 base 位姿，
        // 不再被采集窗口期间的运动量污染。
        const auto query_result = collect_and_validate_points(
            request, local_runtime_config_.pointcloud_topic,
            local_runtime_config_.collect_duration_sec,
            local_runtime_config_.min_accumulated_points);
        if (!query_result.ok) {
            response.message = query_result.error_message;
            return;
        }

        auto odom_to_base_ref = Eigen::Isometry3f::Identity();
        auto tf_error = std::string{};
        if (!lookup_odom_to_base_at(
                query_result.reference_stamp, local_safety_config_.tf_lookup_timeout_sec,
                odom_to_base_ref, tf_error)) {
            // 不 fallback 到 latest：local 的目标是保护导航，不是尽可能成功。
            response.message = "failed to query odom->base at reference_stamp: " + tf_error;
            RCLCPP_WARN(
                node_.get_logger(),
                "local: ref_stamp tf lookup failed (frames=%zu, timeout=%.3fs) | %s",
                query_result.frame_count, local_safety_config_.tf_lookup_timeout_sec,
                tf_error.c_str());
            return;
        }

        RCLCPP_INFO(
            node_.get_logger(),
            "local: snapshot frames=%zu points=%zu ref_stamp=%.3f ref_pose=(%.2f,%.2f,%.1fdeg)",
            query_result.frame_count, query_result.cloud->size(),
            query_result.reference_stamp.seconds(), odom_to_base_ref.translation().x(),
            odom_to_base_ref.translation().y(), extract_yaw_deg(odom_to_base_ref));

        const auto matches = query_sc_matches(
            query_result.cloud, odom_to_base_ref, local_registration_config_.sc_top_k);
        if (matches.empty()) {
            response.message = "SC no match";
            return;
        }

        const auto user_world_to_base = tools::pose_to_isometry(request.initial_guess_world_base);
        auto last_msg = std::string{"local: all SC seeds failed"};

        for (std::size_t i = 0; i < matches.size(); ++i) {
            switch (try_local_seed(
                i, matches[i], query_result.cloud, odom_to_base_ref, user_world_to_base, response,
                last_msg)) {
            case LocalSeedOutcome::Accepted: return;
            case LocalSeedOutcome::Abort: return;
            case LocalSeedOutcome::Rejected: continue;
            }
        }

        response.message = std::move(last_msg);
    }

    /// 把一次 wide 请求中的 SC 全局重定位上下文打包，避免方法签名失控。
    struct WideRequestCtx {
        rmcs_relocation::srv::Relocalize::Response& response;
        std::shared_ptr<PointCloud> query_cloud;
        RegistrationPrior registration_prior;
        WideRegistrationConfig config;
    };

    /// WIDE 只信任 SC seed：不额外生成 seed，也不做 yaw sweep。
    auto make_wide_sc_config() const -> WideRegistrationConfig { return wide_registration_config_; }

    /// 完成一次 wide attempt：算 estimated → validator → 装填 response。
    /// 验收通过返回 true（同时 publish world_to_odom 和置 response.success）。
    auto finalize_wide_attempt(WideRequestCtx& ctx, const RegistrationResult& attempt) -> bool {
        auto odom_to_base_after = Eigen::Isometry3f::Identity();
        if (!lookup_odom_to_base(odom_to_base_after)) {
            ctx.response.message = "failed to query odom->base after registration";
            return false;
        }

        const auto world_to_base_estimated = attempt.world_to_odom * odom_to_base_after;
        const auto validation =
            validator_->evaluate_wide(world_to_base_estimated, attempt.score, attempt.inlier_ratio);

        ctx.response.estimated_world_base = tools::isometry_to_pose(world_to_base_estimated);
        ctx.response.world_to_odom = tools::isometry_to_transform(attempt.world_to_odom);
        ctx.response.fitness_score = static_cast<float>(attempt.score);
        ctx.response.within_field_bounds = validation.within_bounds;
        ctx.response.confidence = validation.confidence;

        if (!validation.accepted) {
            ctx.response.message = build_validation_failure_message("wide", validation, true);
            RCLCPP_WARN(
                node_.get_logger(),
                "wide: SC rejected estimated=(%.2f,%.2f,%.1fdeg) score=%.4f inlier=%.3f | %s",
                world_to_base_estimated.translation().x(),
                world_to_base_estimated.translation().y(), extract_yaw_deg(world_to_base_estimated),
                attempt.score, attempt.inlier_ratio, ctx.response.message.c_str());
            return false;
        }

        log_wide_accepted(world_to_base_estimated, attempt);
        update_and_publish_world_to_odom(attempt.world_to_odom);
        ctx.response.success = true;
        ctx.response.message = "ok";
        return true;
    }

    /// 跑 SC seeds 走 wide pipeline：任何失败都直接返回，决策端负责重试。
    auto try_wide_sc_path(WideRequestCtx& ctx, const Eigen::Isometry3f& odom_to_base_before)
        -> bool {
        if (!scan_context_available_) {
            ctx.response.message = "SC unavailable";
            return false;
        }

        const auto matches = query_sc_matches(
            ctx.query_cloud, odom_to_base_before, wide_registration_config_.sc_top_k);
        if (matches.empty()) {
            ctx.response.message = "SC no match";
            return false;
        }

        auto seeds = std::vector<Eigen::Isometry3f>{};
        seeds.reserve(matches.size());
        for (const auto& match : matches)
            seeds.push_back(sc_match_to_seed(match, odom_to_base_before));

        RCLCPP_INFO(
            node_.get_logger(), "wide: SC returned %zu candidates (top sc_score=%.4f)",
            matches.size(), matches.front().sc_score);
        RCLCPP_INFO(
            node_.get_logger(),
            "wide: SC cfg yaw=±%.1fdeg coarse_step=%.1fdeg "
            "score_th=%.3f cand=%zu iter=%d/%d max_corr=%.1fm",
            ctx.config.yaw_window_deg, ctx.config.coarse_yaw_step_deg,
            ctx.config.coarse_score_threshold, ctx.config.max_candidate_count,
            ctx.config.coarse_iterations, ctx.config.precise_iterations,
            ctx.config.max_correspondence_distance_m);
        for (std::size_t i = 0; i < matches.size(); ++i) {
            const auto& match = matches[i];
            RCLCPP_INFO(
                node_.get_logger(), "wide: SC[%zu] pos=(%.2f,%.2f) yaw=%.1fdeg score=%.4f", i,
                match.world_position.x(), match.world_position.y(), match.yaw_deg, match.sc_score);
        }

        const auto runner = tools::WideSeedRunner{
            common_registration_config_, ctx.config, ctx.query_cloud, map_voxelized_cloud_,
            ctx.registration_prior};
        if (!runner.valid()) {
            ctx.response.message = "wide query preprocess failed";
            return false;
        }

        auto last_message = std::string{"no seed converged"};
        for (std::size_t i = 0; i < seeds.size(); ++i) {
            RCLCPP_INFO(node_.get_logger(), "wide: try SC seed[%zu/%zu]", i + 1, seeds.size());

            auto registration_result = RegistrationResult{};
            if (!runner.run(seeds[i], registration_result)) {
                last_message = "SC seed[" + std::to_string(i) + "] registration failed";
                continue;
            }

            if (finalize_wide_attempt(ctx, registration_result))
                return true;
            last_message = ctx.response.message;
        }

        ctx.response.message = std::string{"wide SC failed: "} + last_message;
        return false;
    }

    void log_wide_accepted(
        const Eigen::Isometry3f& world_to_base_estimated, const RegistrationResult& attempt) const {
        const auto estimated_yaw_deg = extract_yaw_deg(world_to_base_estimated);
        RCLCPP_INFO(
            node_.get_logger(),
            "wide: accepted path=scan_context estimated=(%.2f,%.2f,%.1fdeg) score=%.4f inlier=%.3f",
            world_to_base_estimated.translation().x(), world_to_base_estimated.translation().y(),
            estimated_yaw_deg, attempt.score, attempt.inlier_ratio);
    }

    /**
     * @brief MODE_WIDE：纯 SC 全局重定位。SC 不可用 / 无匹配 / 全失败时直接返回失败，由决策端重试。
     */
    void handle_wide_relocalize(
        const rmcs_relocation::srv::Relocalize::Request& request,
        rmcs_relocation::srv::Relocalize::Response& response) {
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
            wide_runtime_config_.collect_duration_sec, wide_runtime_config_.min_accumulated_points);
        if (!query_result.ok) {
            response.message = query_result.error_message;
            return;
        }

        auto registration_prior = RegistrationPrior{};
        registration_prior.has_prior = false;
        registration_prior.odom_to_base = odom_to_base_before;

        auto ctx = WideRequestCtx{
            .response = response,
            .query_cloud = query_result.cloud,
            .registration_prior = registration_prior,
            .config = make_wide_sc_config(),
        };

        try_wide_sc_path(ctx, odom_to_base_before);
    }

    void handle_relocalize(
        const std::shared_ptr<rmcs_relocation::srv::Relocalize::Request> request,
        std::shared_ptr<rmcs_relocation::srv::Relocalize::Response> response) {
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
        case rmcs_relocation::srv::Relocalize::Request::MODE_INITIAL:
            mode_label = "initial";
            handle_initial_relocalize(*request, *response);
            break;
        case rmcs_relocation::srv::Relocalize::Request::MODE_LOCAL:
            mode_label = "local";
            handle_local_relocalize(*request, *response);
            break;
        case rmcs_relocation::srv::Relocalize::Request::MODE_WIDE:
            mode_label = "wide";
            handle_wide_relocalize(*request, *response);
            break;
        default: response->message = "unsupported relocalization mode"; break;
        }
        log_if_failed(mode_label, *request, *response);
    }

    /// 失败时打一行带上下文的 warn（mode + msg + prior 位置），由 dispatcher 在 handler 返回后调用
    void log_if_failed(
        std::string_view mode, const rmcs_relocation::srv::Relocalize::Request& request,
        const rmcs_relocation::srv::Relocalize::Response& response) const {
        if (!log_failure_details_ || response.success)
            return;
        const auto& p = request.initial_guess_world_base.position;
        RCLCPP_WARN(
            node_.get_logger(),
            "relocalize_failed mode=%.*s prior=(%.2f,%.2f,%.2f) score=%.4f conf=%.3f | %s",
            static_cast<int>(mode.size()), mode.data(), p.x, p.y, p.z,
            static_cast<double>(response.fitness_score), static_cast<double>(response.confidence),
            response.message.c_str());
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
    LocalSafetyConfig local_safety_config_{};
    WideRuntimeConfig wide_runtime_config_{};

    CommonRegistrationConfig common_registration_config_{};
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
    std::shared_ptr<PointCloud> map_cloud_;           ///< 原始全图（仅用于 SC map_hash 校验）
    std::shared_ptr<PointCloud> map_voxelized_cloud_; ///< 启动时预处理后的 GICP target

    std::unique_ptr<Collector> collector_;
    std::unique_ptr<Validator> validator_;

    rclcpp::CallbackGroup::SharedPtr service_group_;
    rclcpp::CallbackGroup::SharedPtr pointcloud_group_;
    rclcpp::Service<rmcs_relocation::srv::Relocalize>::SharedPtr relocalize_service_;

    std::shared_ptr<rclcpp::TimerBase> tf_publish_timer_;

    mutable tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    mutable std::mutex state_mutex_;
    bool busy_ = false;
    geometry_msgs::msg::TransformStamped current_world_to_odom_;

    /// 上一次 local 成功 publish 的 ROS 时刻。配合 min_accept_interval_sec 限速。
    /// 仅由 service 回调线程访问（MutuallyExclusive group + BusyGuard 串行化），无需上锁。
    rclcpp::Time last_local_accept_time_{0, 0, RCL_ROS_TIME};
};

RelocalizationServer::RelocalizationServer()
    : Node("rmcs_relocation", rclcpp::NodeOptions{})
    , pimpl_(std::make_unique<Impl>(*this)) {}

RelocalizationServer::~RelocalizationServer() = default;

} // namespace rmcs::location

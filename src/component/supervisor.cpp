/**
 * @file supervisor.cpp
 * @brief 位置重定位管理器实现
 *
 * 该组件负责管理机器人的重定位过程，包括初始重定位和丢失重定位。
 * 通过监控游戏状态、机器人健康状态和血量变化，智能触发重定位请求。
 *
 * @author RMCS Development Team
 */

#include "component/supervisor.hpp"
#include "tools/geometry_tools.hpp"
#include "tools/numeric_tools.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/msg/location_health.hpp>
#include <rmcs_msgs/robot_color.hpp>
#include <rmcs_msgs/robot_id.hpp>
#include <rmcs_msgs/srv/relocalize.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rmcs::location {

namespace tools {
auto read_pose_parameter(
    rclcpp::Node& node, const std::string& prefix, const geometry_msgs::msg::Pose& defaults)
    -> geometry_msgs::msg::Pose;
} // namespace tools

/**
 * @brief 监督器内部实现
 *
 * 管理整个自动重定位状态机，包括：
 * - 比赛倒计时边缘触发的初始重定位（支持重试）
 * - 血量复活/健康状态持续不健康触发的丢失重定位
 * - 作为 rmcs_executor 插件组件运行，通过服务客户端异步调用 relocalize 服务
 */
struct Supervisor::Impl {
    template <typename T>
    using InputInterface = rmcs_executor::Component::InputInterface<T>;

    /**
     * @brief 重定位阶段状态机
     *
     * IDLE: 未开始，等待触发
     * PENDING_INITIAL: 已武装，等待重试间隔后发送请求
     * INITIAL_REQUEST_IN_FLIGHT: 初始重定位请求进行中
     * INITIALIZED: 初始重定位成功完成
     * LOST_REQUEST_IN_FLIGHT_FROM_IDLE/FROM_INITIALIZED: 丢失重定位请求进行中
     */
    enum class Phase {
        IDLE,
        PENDING_INITIAL,
        INITIAL_REQUEST_IN_FLIGHT,
        INITIALIZED,
        LOST_REQUEST_IN_FLIGHT_FROM_IDLE,
        LOST_REQUEST_IN_FLIGHT_FROM_INITIALIZED,
    };

    using TimePoint = std::chrono::steady_clock::time_point;

    /**
     * @brief 构造内部实现
     *
     * 加载全部运行时参数（重试间隔、超时、冷却时间等），
     * 注册输入接口、订阅健康状态话题、初始化 TF 缓存和 relocalize 服务客户端。
     */
    explicit Impl(Supervisor& owner)
        : owner_(owner)
        , service_name_(
              owner_.get_parameter_or<std::string>("service_name", "/rmcs_relocation/relocalize"))
        , world_frame_(owner_.get_parameter_or<std::string>("world_frame", "world"))
        , odom_frame_(owner_.get_parameter_or<std::string>("odom_frame", "odom"))
        , base_frame_(owner_.get_parameter_or<std::string>("base_frame", "base_link"))
        , initial_pointcloud_topic_(owner_.get_parameter_or<std::string>(
              "initial.pointcloud_topic", "/cloud_registered_undistort"))
        , initial_collect_duration_sec_(
              owner_.get_parameter_or<double>("initial.collect_duration_sec", 2.0))
        , lost_pointcloud_topic_(owner_.get_parameter_or<std::string>(
              "lost.pointcloud_topic", "/cloud_registered_undistort"))
        , lost_collect_duration_sec_(
              owner_.get_parameter_or<double>("lost.collect_duration_sec", 2.0))
        , retry_interval_sec_(owner_.get_parameter_or<double>("retry_interval_sec", 2.0))
        , request_timeout_sec_(
              std::max(0.1, owner_.get_parameter_or<double>("request_timeout_sec", 8.0)))
        , max_attempt_count_(
              std::max<std::size_t>(
                  1, static_cast<std::size_t>(owner_.get_parameter_or<int>("max_retry_count", 3))))
        , health_unhealthy_dwell_sec_(
              owner_.get_parameter_or<double>("health_unhealthy_dwell_sec", 0.8))
        , lost_cooldown_sec_(owner_.get_parameter_or<double>("lost_cooldown_sec", 3.0))
        , lost_max_consecutive_failures_(
              std::max<std::size_t>(
                  1, static_cast<std::size_t>(
                         owner_.get_parameter_or<int>("lost_max_consecutive_failures", 5))))
        , lost_sigma_xy_base_m_(owner_.get_parameter_or<double>("lost_sigma_xy_base_m", 1.0))
        , lost_sigma_yaw_base_deg_(
              owner_.get_parameter_or<double>("lost_sigma_yaw_base_deg", 20.0))
        , relocalize_client_(owner_.create_client<rmcs_msgs::srv::Relocalize>(service_name_))
        , tf_buffer_(owner_.get_clock())
        , tf_listener_(std::make_unique<tf2_ros::TransformListener>(tf_buffer_, &owner_, false)) {
        default_initial_guess_ =
            tools::read_pose_parameter(owner_, "default_initial_guess", geometry_msgs::msg::Pose{});
        opposite_initial_guess_ = tools::read_pose_parameter(
            owner_, "opposite_initial_guess", geometry_msgs::msg::Pose{});

        owner_.register_input("/referee/game/stage", game_stage_);
        owner_.register_input("/referee/id", robot_id_);
        owner_.register_input("/referee/current_hp", current_hp_);

        health_subscription_ = owner_.create_subscription<rmcs_msgs::msg::LocationHealth>(
            "/rmcs_relocation/health", rclcpp::QoS(10),
            [this](const rmcs_msgs::msg::LocationHealth::SharedPtr message) {
                if (!message)
                    return;

                const auto now = std::chrono::steady_clock::now();
                auto lock = std::scoped_lock{health_mutex_};

                if (message->state == rmcs_msgs::msg::LocationHealth::STATE_UNHEALTHY) {
                    if (latest_health_state_ != rmcs_msgs::msg::LocationHealth::STATE_UNHEALTHY)
                        health_unhealthy_since_ = now;
                } else {
                    health_unhealthy_since_.reset();
                }

                latest_health_state_ = message->state;
                health_ready_ = true;
            });
    }

    /**
     * @brief 每个循环周期的主入口
     *
     * 顺序执行：HP 复活检测 → 检查进行中的请求是否完成/超时 → 尝试初始重定位 → 尝试丢失重定位。
     * 有请求进行中时跳过新的触发尝试，避免并发请求。
     */
    void update() {
        if (!all_inputs_ready())
            return;

        const auto now = std::chrono::steady_clock::now();

        // 更新复活检测
        update_hp_revival_detection();
        // 检查是否有正在进行中的请求
        check_inflight_request(now);

        if (!is_request_in_flight()) {
            //倒计时触发initial
            arm_initial_relocalize_on_countdown_edge(now);
            try_initial_relocalize(now);

            //还没有request直接进行lost重定位
            if (!is_request_in_flight())
                try_lost_relocalize(now);
        }

        previous_stage_ = *game_stage_;
    }

    auto all_inputs_ready() const -> bool {
        return game_stage_.ready() && robot_id_.ready() && current_hp_.ready();
    }

    auto is_request_in_flight() const -> bool {
        return phase_ == Phase::INITIAL_REQUEST_IN_FLIGHT
            || phase_ == Phase::LOST_REQUEST_IN_FLIGHT_FROM_IDLE
            || phase_ == Phase::LOST_REQUEST_IN_FLIGHT_FROM_INITIALIZED;
    }

    auto is_initialized() const -> bool {
        return phase_ == Phase::INITIALIZED
            || phase_ == Phase::LOST_REQUEST_IN_FLIGHT_FROM_INITIALIZED;
    }

    auto is_initial_relocalize_armed() const -> bool {
        return phase_ == Phase::PENDING_INITIAL || phase_ == Phase::INITIAL_REQUEST_IN_FLIGHT;
    }

    /**
     * @brief 在 COUNTDOWN 阶段上升沿武装初始重定位
     *
     * 检测游戏从非 COUNTDOWN 进入 COUNTDOWN 的边缘，设置 PENDING_INITIAL 阶段并初始化重试计数。
     */
    void arm_initial_relocalize_on_countdown_edge(const TimePoint& now) {
        const auto current_stage = *game_stage_;
        const auto started_edge = current_stage == rmcs_msgs::GameStage::COUNTDOWN
                               && previous_stage_ != rmcs_msgs::GameStage::COUNTDOWN;

        if (started_edge && !is_initialized()) {
            phase_ = Phase::PENDING_INITIAL;
            attempt_count_ = 0;
            next_retry_time_ = now;
            RCLCPP_INFO(owner_.get_logger(), "initial relocalization armed on COUNTDOWN edge\n");
        }
    }

    /**
     * @brief 检测血量复活事件
     *
     * 每一帧比较当前 HP 与上一帧 HP，若由 0 变为正数则标记为本周期复活。
     */
    void update_hp_revival_detection() {
        const auto current_hp = static_cast<std::uint16_t>(*current_hp_);
        if (!previous_hp_.has_value()) {
            previous_hp_ = current_hp;
            hp_revived_this_cycle_ = false;
            return;
        }

        hp_revived_this_cycle_ = *previous_hp_ == 0 && current_hp > 0;
        previous_hp_ = current_hp;
    }

    //检查进行中的重定位请求是否完成或超时
    void check_inflight_request(const TimePoint& now) {
        using namespace std::chrono_literals;

        if (!is_request_in_flight())
            return;

        const auto mode = pending_request_mode_;
        const auto lost_from_initialized = phase_ == Phase::LOST_REQUEST_IN_FLIGHT_FROM_INITIALIZED;

        //INITIAL 模式失败后保持 PENDING_INITIAL 等待重试；LOST 模式失败后累加连续失败计数。
        const auto restore_phase = [this, mode, lost_from_initialized] {
            if (mode == rmcs_msgs::srv::Relocalize::Request::MODE_INITIAL) {
                phase_ = Phase::PENDING_INITIAL;
                return;
            }
            phase_ = lost_from_initialized ? Phase::INITIALIZED : Phase::IDLE;
        };

        //若 future 已就绪则处理响应（成功/失败恢复阶段），若超过 timeout 则主动取消请求。
        if (pending_future_.wait_for(0s) == std::future_status::ready) {
            auto response = pending_future_.get();
            pending_request_id_ = 0;
            pending_request_send_time_.reset();
            restore_phase();

            /** 
             * @brief 处理重定位请求响应
             *
             * 初始化模式成功：状态设为INITIALIZED，重置重试计时器
             * 丢失模式成功：失败计数器清零，记录使用的配准层级(tier)
             * 保存估计的世界坐标系位姿   
            */
            if (response && response->success) {
                if (mode == rmcs_msgs::srv::Relocalize::Request::MODE_INITIAL) {
                    phase_ = Phase::INITIALIZED;
                    next_retry_time_.reset();
                    RCLCPP_INFO(
                        owner_.get_logger(), "initial relocalization succeeded, score=%.4f\n",
                        response->fitness_score);
                } else {
                    lost_failure_count_ = 0;
                    RCLCPP_INFO(
                        owner_.get_logger(), "lost relocalization succeeded, score=%.4f tier=%u\n",
                        response->fitness_score, response->tier_used);
                }

                last_known_world_base_ = response->estimated_world_base;
                return;
            }

            /**
             * @brief 检测血量复活事件
             *
             * 处理失败情况
             * 初始化模式失败：设置下次重试时间，记录尝试次数
             * 丢失模式失败：增加失败计数器（有上限保护）
             */
            if (mode == rmcs_msgs::srv::Relocalize::Request::MODE_INITIAL) {
                next_retry_time_ = now + tools::as_steady_duration(retry_interval_sec_);
                RCLCPP_WARN(
                    owner_.get_logger(),
                    "initial relocalization failed, waiting for retry (%zu/%zu)\n", attempt_count_,
                    max_attempt_count_);
            } else {
                lost_failure_count_ =
                    std::min(lost_failure_count_ + 1, lost_max_consecutive_failures_);
                RCLCPP_WARN(
                    owner_.get_logger(), "lost relocalization failed, consecutive_failures=%zu\n",
                    lost_failure_count_);
            }
            return;
        }

        // 处理请求超时
        if (pending_request_send_time_.has_value()
            && tools::elapsed_sec(*pending_request_send_time_, now) >= request_timeout_sec_) {
            relocalize_client_->remove_pending_request(pending_request_id_);
            pending_request_id_ = 0;
            pending_request_send_time_.reset();
            restore_phase();

            // 处理超时后的状态更新
            if (mode == rmcs_msgs::srv::Relocalize::Request::MODE_INITIAL) {
                next_retry_time_ = now + tools::as_steady_duration(retry_interval_sec_);
                RCLCPP_WARN(
                    owner_.get_logger(),
                    "initial relocalization request timed out after %.2fs, waiting for retry "
                    "(%zu/%zu)\n",
                    request_timeout_sec_, attempt_count_, max_attempt_count_);
            } else {
                lost_failure_count_ =
                    std::min(lost_failure_count_ + 1, lost_max_consecutive_failures_);
                RCLCPP_WARN(
                    owner_.get_logger(),
                    "lost relocalization request timed out after %.2fs, consecutive_failures=%zu\n",
                    request_timeout_sec_, lost_failure_count_);
            }
        }
    }

    /**
     * @brief 尝试发送初始重定位请求
     *
     * 仅在 PENDING_INITIAL 阶段且重试间隔已过时触发。达到最大尝试次数后放弃。
     * 服务不可用时计数为一次失败并进入重试等待。
     */
    void try_initial_relocalize(const TimePoint& now) {
        if (phase_ != Phase::PENDING_INITIAL)
            return;

        if (attempt_count_ >= max_attempt_count_) {
            phase_ = Phase::IDLE;
            next_retry_time_.reset();
            RCLCPP_ERROR(
                owner_.get_logger(), "initial relocalization exhausted retries (%zu attempts)\n",
                attempt_count_);
            return;
        }

        if (next_retry_time_.has_value() && now < *next_retry_time_)
            return;

        if (!try_send_initial_request(now)) {
            ++attempt_count_;
            next_retry_time_ = now + tools::as_steady_duration(retry_interval_sec_);
            RCLCPP_WARN(
                owner_.get_logger(), "relocalization service unavailable (%zu/%zu)\n", attempt_count_,
                max_attempt_count_);
            return;
        }

        phase_ = Phase::INITIAL_REQUEST_IN_FLIGHT;
        ++attempt_count_;

        RCLCPP_INFO(
            owner_.get_logger(), "sent initial relocalization request (%zu/%zu)\n", attempt_count_,
            max_attempt_count_);
    }

    /**
     * @brief 尝试发送丢失重定位请求
     *
     * 两种触发条件互斥且本周期只取其一：
     * 1. HP 复活事件 — 立即触发，清零失败计数
     * 2. 健康状态持续 UNHEALTHY 且冷却时间已过
     */
    void try_lost_relocalize(const TimePoint& now) {
        if (is_initial_relocalize_armed())
            return;

        if (hp_revived_this_cycle_) {
            lost_failure_count_ = 0;
            try_send_lost_request("hp revival", now);
            return;
        }

        if (is_unhealthy_relocalize_ready(now) && is_lost_cooldown_ready(now))
            try_send_lost_request("health unhealthy", now);
    }

    auto is_unhealthy_relocalize_ready(const TimePoint& now) const -> bool {
        auto lock = std::scoped_lock{health_mutex_};
        if (!health_ready_
            || latest_health_state_ != rmcs_msgs::msg::LocationHealth::STATE_UNHEALTHY
            || !health_unhealthy_since_.has_value()) {
            return false;
        }
        return tools::elapsed_sec(*health_unhealthy_since_, now) >= health_unhealthy_dwell_sec_;
    }

    auto is_lost_cooldown_ready(const TimePoint& now) const -> bool {
        return !last_lost_attempt_time_.has_value()
            || tools::elapsed_sec(*last_lost_attempt_time_, now) >= lost_cooldown_sec_;
    }

    /**
     * @brief 根据队伍颜色选择初始猜测位姿
     */
    auto select_initial_guess() const -> geometry_msgs::msg::Pose {
        return robot_id_->color() == rmcs_msgs::RobotColor::BLUE ? opposite_initial_guess_
                                                                 : default_initial_guess_;
    }

    /**
     * @brief 解析丢失重定位的先验位姿
     *
     * 优先使用上次成功重定位时的 world->base 位姿；回退方案通过 TF 链 world->odom->base 推算当前位姿。
     */
    auto try_resolve_lost_prior(geometry_msgs::msg::Pose& prior_pose) -> bool {
        if (last_known_world_base_.has_value()) {
            prior_pose = *last_known_world_base_;
            return true;
        }

        try {
            const auto world_to_odom =
                tf_buffer_.lookupTransform(world_frame_, odom_frame_, tf2::TimePointZero);
            const auto odom_to_base =
                tf_buffer_.lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);

            const auto world_to_base = tools::transform_to_isometry(world_to_odom.transform)
                                     * tools::transform_to_isometry(odom_to_base.transform);
            prior_pose = tools::isometry_to_pose(world_to_base);
            return true;
        } catch (const tf2::TransformException& error) {
            RCLCPP_WARN(
                owner_.get_logger(), "failed to resolve lost prior from TF: %s\n", error.what());
            return false;
        }
    }

    /**
     * @brief 构造初始重定位请求（MODE_INITIAL）
     */
    auto create_initial_request() const -> std::shared_ptr<rmcs_msgs::srv::Relocalize::Request> {
        auto request = std::make_shared<rmcs_msgs::srv::Relocalize::Request>();
        request->mode = rmcs_msgs::srv::Relocalize::Request::MODE_INITIAL;
        request->initial_guess_world_base = select_initial_guess();
        request->pointcloud_topic = initial_pointcloud_topic_;
        request->collect_duration_sec = static_cast<float>(initial_collect_duration_sec_);
        request->prior_sigma_xy_m = 0.0F;
        request->prior_sigma_yaw_deg = 0.0F;
        return request;
    }

    /**
     * @brief 构造丢失重定位请求（MODE_LOST）
     *
     * @param prior_pose   先验 world->base 位姿
     * @param sigma_xy    平移不确定度（指数放大后的值）
     * @param sigma_yaw   yaw 不确定度（指数放大后的值）
     */
    auto create_lost_request(
        const geometry_msgs::msg::Pose& prior_pose, double sigma_xy, double sigma_yaw) const
        -> std::shared_ptr<rmcs_msgs::srv::Relocalize::Request> {
        auto request = std::make_shared<rmcs_msgs::srv::Relocalize::Request>();
        request->mode = rmcs_msgs::srv::Relocalize::Request::MODE_LOST;
        request->initial_guess_world_base = tools::normalize_pose(prior_pose);
        request->pointcloud_topic = lost_pointcloud_topic_;
        request->collect_duration_sec = static_cast<float>(lost_collect_duration_sec_);
        request->prior_sigma_xy_m = static_cast<float>(sigma_xy);
        request->prior_sigma_yaw_deg = static_cast<float>(sigma_yaw);
        return request;
    }

    /**
     * @brief 发送重定位请求
     *
     * 检查服务是否就绪，通过 RequestBuilder 回调构造请求对象，调用异步发送并保存 future/request_id。
     * @return 服务就绪且发送成功返回 true，否则 false
     */
    template <typename RequestBuilder>
    auto send_request(RequestBuilder&& build, const TimePoint& now) -> bool {
        if (!relocalize_client_->service_is_ready()
            && !relocalize_client_->wait_for_service(std::chrono::seconds(0))) {
            return false;
        }

        auto request = std::forward<RequestBuilder>(build)();
        if (!request)
            return false;

        auto future_result = relocalize_client_->async_send_request(request);
        pending_future_ = future_result.future.share();
        pending_request_id_ = future_result.request_id;
        pending_request_send_time_ = now;
        pending_request_mode_ = request->mode;
        return true;
    }

    /**
     * @brief 发送初始重定位请求
     *
     * 通过 send_request 模板发送 MODE_INITIAL 请求，sigma 参数均为 0（初始模式不使用先验不确定性）。
     */
    auto try_send_initial_request(const TimePoint& now) -> bool {
        return send_request([this] { return create_initial_request(); }, now);
    }

    /**
     * @brief 发送丢失重定位请求
     *
     * 先解析先验位姿，根据连续失败次数指数放大 sigma 参数（平移上限 12m，yaw 上限 180°），
     * 然后通过 send_request 模板发送 MODE_LOST 请求。
     *
     * @param trigger_reason 触发原因（"hp revival" 或 "health unhealthy"），仅用于日志
     */
    auto try_send_lost_request(const std::string& trigger_reason, const TimePoint& now) -> bool {
        auto prior_pose = geometry_msgs::msg::Pose{};
        if (!try_resolve_lost_prior(prior_pose)) {
            last_lost_attempt_time_ = now;
            return false;
        }

        const auto pow_exp = std::min<std::size_t>(lost_failure_count_, 8);
        const auto scale = std::pow(2.0, static_cast<double>(pow_exp));
        const auto sigma_xy = std::min(12.0, lost_sigma_xy_base_m_ * scale);
        const auto sigma_yaw = std::min(180.0, lost_sigma_yaw_base_deg_ * scale);

        if (!send_request(
                [this, &prior_pose, sigma_xy, sigma_yaw] {
                    return create_lost_request(prior_pose, sigma_xy, sigma_yaw);
                },
                now)) {
            last_lost_attempt_time_ = now;
            RCLCPP_WARN(owner_.get_logger(), "relocalization service unavailable for lost request\n");
            return false;
        }

        phase_ = is_initialized() ? Phase::LOST_REQUEST_IN_FLIGHT_FROM_INITIALIZED
                                  : Phase::LOST_REQUEST_IN_FLIGHT_FROM_IDLE;
        last_lost_attempt_time_ = now;

        RCLCPP_INFO(
            owner_.get_logger(),
            "sent lost relocalization request (trigger=%s, sigma_xy=%.2f, sigma_yaw=%.2f)\n",
            trigger_reason.c_str(), sigma_xy, sigma_yaw);
        return true;
    }

    Supervisor& owner_;

    InputInterface<rmcs_msgs::GameStage> game_stage_;
    InputInterface<rmcs_msgs::RobotId> robot_id_;
    InputInterface<std::uint16_t> current_hp_;

    std::string service_name_;
    std::string world_frame_;
    std::string odom_frame_;
    std::string base_frame_;

    std::string initial_pointcloud_topic_;
    double initial_collect_duration_sec_;
    std::string lost_pointcloud_topic_;
    double lost_collect_duration_sec_;
    double retry_interval_sec_;
    double request_timeout_sec_;
    std::size_t max_attempt_count_;
    double health_unhealthy_dwell_sec_;
    double lost_cooldown_sec_;
    std::size_t lost_max_consecutive_failures_;
    double lost_sigma_xy_base_m_;
    double lost_sigma_yaw_base_deg_;

    geometry_msgs::msg::Pose default_initial_guess_{};
    geometry_msgs::msg::Pose opposite_initial_guess_{};

    std::shared_ptr<rclcpp::Client<rmcs_msgs::srv::Relocalize>> relocalize_client_;
    rclcpp::Client<rmcs_msgs::srv::Relocalize>::SharedFuture pending_future_;
    int64_t pending_request_id_ = 0;
    std::optional<TimePoint> pending_request_send_time_{};
    std::uint8_t pending_request_mode_ = rmcs_msgs::srv::Relocalize::Request::MODE_INITIAL;

    rclcpp::Subscription<rmcs_msgs::msg::LocationHealth>::SharedPtr health_subscription_;
    mutable std::mutex health_mutex_;
    std::uint8_t latest_health_state_ = rmcs_msgs::msg::LocationHealth::STATE_HEALTHY;
    std::optional<TimePoint> health_unhealthy_since_{};
    bool health_ready_ = false;

    mutable tf2_ros::Buffer tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    rmcs_msgs::GameStage previous_stage_ = rmcs_msgs::GameStage::UNKNOWN;
    Phase phase_ = Phase::IDLE;
    std::size_t attempt_count_ = 0;
    std::optional<TimePoint> next_retry_time_{};

    std::optional<std::uint16_t> previous_hp_{};
    bool hp_revived_this_cycle_ = false;

    std::size_t lost_failure_count_ = 0;
    std::optional<TimePoint> last_lost_attempt_time_{};
    std::optional<geometry_msgs::msg::Pose> last_known_world_base_{};
};

Supervisor::Supervisor()
    : Node(
          get_component_name(),
          rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    , pimpl_(std::make_unique<Impl>(*this)) {}

Supervisor::~Supervisor() = default;

void Supervisor::update() { pimpl_->update(); }

} // namespace rmcs::location

PLUGINLIB_EXPORT_CLASS(rmcs::location::Supervisor, rmcs_executor::Component)

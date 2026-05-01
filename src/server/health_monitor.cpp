/**
 * @file health_monitor.cpp
 * @brief 位置健康监控器实现
 *
 * 实现了位置系统的健康状态监控功能，通过分析点云配准残差和内点比例
 * 来评估定位系统的可靠性。支持多状态转换和防抖动机制。
 *
 * @author RMCS Development Team
 */

#include "health_monitor.hpp"

#include "tools/numeric_tools.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include <pcl/common/transforms.h>

namespace rmcs::location {

namespace {

/**
 * @brief 计算浮点数向量的中位数
 *
 * 使用 std::nth_element 部分排序，偶数长度时取中间两个元素的均值。
 */
auto median_of(std::vector<float>& values) -> float {
    if (values.empty())
        return std::numeric_limits<float>::infinity();

    const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
    std::nth_element(values.begin(), middle, values.end());

    if (values.size() % 2 != 0)
        return *middle;

    const auto lower_middle = *std::max_element(values.begin(), middle);
    return 0.5F * (lower_middle + *middle);
}

} // namespace

/**
 * @brief 健康监控器内部实现
 *
 * 维护一个三状态健康状态机（HEALTHY → WARNING → UNHEALTHY），
 * 通过评估当前点云与地图 KD 树的匹配残差和内点比例来判定状态转换，
 * 每个状态转换均带有防抖动（dwell time）机制。
 */
struct HealthMonitor::Impl {
    using TimePoint = std::chrono::steady_clock::time_point;

    explicit Impl(HealthRuntimeConfig config)
        : config_(sanitize_config(std::move(config))) {}

    /**
     * @brief 对配置进行一次性的合法性清理和回退值填充
     *
     * 在构造时调用，后续 evaluate() 不再需要重复 sanitize。
     */
    static auto sanitize_config(HealthRuntimeConfig config) -> HealthRuntimeConfig {
        config.rate_hz = std::max(1.0, tools::sanitize_non_negative(config.rate_hz, 5.0));
        config.sample_points = tools::sanitize_positive_int(config.sample_points, 500);

        config.warn_threshold_m = tools::sanitize_non_negative(config.warn_threshold_m, 0.25);
        config.lost_threshold_m = tools::sanitize_non_negative(config.lost_threshold_m, 0.45);
        config.min_inlier_ratio = tools::sanitize_non_negative(config.min_inlier_ratio, 0.30);

        config.warn_dwell_sec = tools::sanitize_non_negative(config.warn_dwell_sec, 0.6);
        config.lost_dwell_sec = tools::sanitize_non_negative(config.lost_dwell_sec, 1.0);

        config.recover_margin_m = tools::sanitize_non_negative(config.recover_margin_m, 0.05);
        config.recover_dwell_sec = tools::sanitize_non_negative(config.recover_dwell_sec, 2.0);

        config.inlier_distance_m = tools::sanitize_non_negative(config.inlier_distance_m, 0.5);
        return config;
    }

    /**
     * @brief 接收一帧已转换到 odom 坐标系的点云
     *
     * 线程安全，保存最近一帧供 evaluate() 周期消费。
     */

    //接收并存储里程计坐标系下的点云数据
    void ingest_cloud_odom(const std::shared_ptr<PointCloud>& cloud_odom) {
        if (!cloud_odom || cloud_odom->empty())
            return;

        auto lock = std::scoped_lock{cloud_mutex_};
        latest_cloud_odom_ = cloud_odom;
        has_latest_cloud_ = true;
    }

    /**
     * @brief 健康状态评估
     *
     * 输入：地图KD树、变换矩阵、时间戳
     * 输出：定位健康状态消息
     * 初始化残差中位数和内点比例变量
     */
    auto evaluate(
        const pcl::KdTreeFLANN<Point>& map_kdtree, bool map_kdtree_ready,
        const Eigen::Isometry3f& world_to_odom, const rclcpp::Time& stamp)
        -> rmcs_msgs::msg::LocationHealth {
        auto residual_median_m = 0.0F;
        auto inlier_ratio = 0.0F;

        //计算健康指标->估计点云与地图的匹配质量
        const auto metrics_ok = estimate_health_metrics(
            map_kdtree, map_kdtree_ready, world_to_odom, residual_median_m, inlier_ratio);

        if (metrics_ok) {
            last_residual_median_m_ = residual_median_m;
            last_inlier_ratio_ = inlier_ratio;

            //状态转换阈值
            const auto now_steady = std::chrono::steady_clock::now();
            const auto recover_threshold =
                std::max(0.0, config_.warn_threshold_m - config_.recover_margin_m);

            /**
             * @brief HEALTHY状态评估
             *
             * 条件：残差超过警告阈值
             * 机制：需要持续超过warn_dwell_sec秒才转换到警告状态
             * 防抖动：如果残差恢复正常，重置计时器
             */
            if (state_ == rmcs_msgs::msg::LocationHealth::STATE_HEALTHY) {
                if (residual_median_m > config_.warn_threshold_m) {
                    if (!above_warn_since_.has_value())
                        above_warn_since_ = now_steady;

                    if (tools::elapsed_sec(*above_warn_since_, now_steady)
                        >= config_.warn_dwell_sec) {
                        state_ = rmcs_msgs::msg::LocationHealth::STATE_WARNING;
                        above_warn_since_.reset();
                    }
                } else {
                    above_warn_since_.reset();
                }
            //WARNING状态评估
            } else if (state_ == rmcs_msgs::msg::LocationHealth::STATE_WARNING) {
                const auto should_lost = residual_median_m > config_.lost_threshold_m
                                      || inlier_ratio < config_.min_inlier_ratio;
                if (should_lost) {
                    if (!above_lost_since_.has_value())
                        above_lost_since_ = now_steady;

                    if (tools::elapsed_sec(*above_lost_since_, now_steady)
                        >= config_.lost_dwell_sec) {
                        state_ = rmcs_msgs::msg::LocationHealth::STATE_UNHEALTHY;
                        above_lost_since_.reset();
                        below_recover_since_.reset();
                    }
                } else {
                    above_lost_since_.reset();
                }
                //从WARNING恢复到HEALTHY
                const auto should_recover = residual_median_m < recover_threshold
                                         && inlier_ratio >= config_.min_inlier_ratio;
                try_transition_recover(
                    should_recover, below_recover_since_, config_.recover_dwell_sec,
                    rmcs_msgs::msg::LocationHealth::STATE_HEALTHY, now_steady);
            } else {//UNHEALTHY状态处理
                const auto should_recover = residual_median_m < recover_threshold
                                         && inlier_ratio >= config_.min_inlier_ratio;
                try_transition_recover(
                    should_recover, below_recover_since_, config_.recover_dwell_sec,
                    rmcs_msgs::msg::LocationHealth::STATE_WARNING, now_steady);
            }
        }

        //返回健康状态消息
        auto health = rmcs_msgs::msg::LocationHealth{};
        health.state = state_;
        health.residual_median_m = last_residual_median_m_;
        health.inlier_ratio = last_inlier_ratio_;
        health.stamp = stamp;
        return health;
    }

    /**
     * @brief 尝试从 WARNING/UNHEALTHY 恢复到较低严重级别
     *
     * 当恢复条件持续满足超过 recover_dwell_sec 时执行状态跃迁。
     * 条件不满足时立即重置计时器。
     *
     * @param should_recover  当前周期恢复条件是否满足
     * @param below_since     恢复条件首次满足的时间点（状态变量）
     * @param dwell_sec       所需的持续满足时长
     * @param target_state    目标状态
     * @param now             当前时间
     * @return 是否在本周期完成了恢复跃迁
     */

    //状态恢复转换实现
    auto try_transition_recover(
        bool should_recover, std::optional<TimePoint>& below_since, double dwell_sec,
        std::uint8_t target_state, const TimePoint& now) -> bool {
        if (!should_recover) {
            below_since.reset();
            return false;
        }

        //如果是第一次满足条件，记录开始时间
        if (!below_since.has_value())
            below_since = now;

        //检查是否持续了足够长的时间
        if (tools::elapsed_sec(*below_since, now) < dwell_sec)
            return false;

        state_ = target_state;
        below_since.reset();
        return true;
    }

    //健康指标估计实现和前置检查
    auto estimate_health_metrics(
        const pcl::KdTreeFLANN<Point>& map_kdtree, bool map_kdtree_ready,
        const Eigen::Isometry3f& world_to_odom, float& residual_median_m, float& inlier_ratio) const
        -> bool {
        if (!map_kdtree_ready)
            return false;

        //获取最新的点云数据
        auto latest_cloud = std::shared_ptr<PointCloud>{};
        {
            auto lock = std::scoped_lock{cloud_mutex_};
            if (!has_latest_cloud_ || !latest_cloud_odom_ || latest_cloud_odom_->empty())
                return false;
            latest_cloud = latest_cloud_odom_;
        }

        //将点云转换到世界坐标系
        auto cloud_world = PointCloud{};
        pcl::transformPointCloud(*latest_cloud, cloud_world, world_to_odom.matrix());
        if (cloud_world.empty())
            return false;

        const auto sample_count =
            std::max(1, std::min(static_cast<int>(cloud_world.size()), config_.sample_points));
        const auto step =
            std::max<std::size_t>(1, cloud_world.size() / static_cast<std::size_t>(sample_count));

        std::vector<float> residuals;
        residuals.reserve(static_cast<std::size_t>(sample_count));//储存每个点的残差

        const auto inlier_distance_threshold = static_cast<float>(config_.inlier_distance_m);

        auto inlier_count = std::size_t{0};
        std::vector<int> indices(1);
        std::vector<float> distances_sq(1);

        //遍历采样点进行匹配计算
        for (std::size_t i = 0;
             i < cloud_world.size() && residuals.size() < static_cast<std::size_t>(sample_count);
             i += step) {
            //在KD树中查找最近邻点
            if (map_kdtree.nearestKSearch(cloud_world.points[i], 1, indices, distances_sq) <= 0)
                continue;

            const auto residual = std::sqrt(distances_sq[0]);
            residuals.push_back(residual);
            if (residual <= inlier_distance_threshold)
                ++inlier_count;
        }

        //计算结果并返回
        if (residuals.empty())
            return false;

        residual_median_m = median_of(residuals);
        inlier_ratio = static_cast<float>(inlier_count) / static_cast<float>(residuals.size());
        return true;
    }

    HealthRuntimeConfig config_{};

    mutable std::mutex cloud_mutex_;
    std::shared_ptr<PointCloud> latest_cloud_odom_;
    bool has_latest_cloud_ = false;

    std::uint8_t state_ = rmcs_msgs::msg::LocationHealth::STATE_HEALTHY;
    std::optional<TimePoint> above_warn_since_{};
    std::optional<TimePoint> above_lost_since_{};
    std::optional<TimePoint> below_recover_since_{};

    float last_residual_median_m_ = std::numeric_limits<float>::infinity();
    float last_inlier_ratio_ = 0.0F;
};

HealthMonitor::HealthMonitor(HealthRuntimeConfig config)
    : pimpl_(std::make_unique<Impl>(std::move(config))) {}

HealthMonitor::~HealthMonitor() = default;

void HealthMonitor::ingest_cloud_odom(const std::shared_ptr<PointCloud>& cloud_odom) {
    pimpl_->ingest_cloud_odom(cloud_odom);
}

auto HealthMonitor::evaluate(
    const pcl::KdTreeFLANN<Point>& map_kdtree, bool map_kdtree_ready,
    const Eigen::Isometry3f& world_to_odom, const rclcpp::Time& stamp)
    -> rmcs_msgs::msg::LocationHealth {
    return pimpl_->evaluate(map_kdtree, map_kdtree_ready, world_to_odom, stamp);
}

} // namespace rmcs::location

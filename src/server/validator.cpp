/**
 * @file validator.cpp
 * @brief 重定位验证器实现
 *
 * 实现了重定位结果的验证逻辑，包括初始重定位和丢失重定位的验证。
 *
 * @author RMCS Development Team
 */

#include "validator.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace rmcs::location {

namespace {

/**
 * @brief 根据重定位层级选择 LOCAL 或 WIDE 对应的阈值
 */
auto select_by_tier(LostTier tier_used, double local_value, double wide_value) -> double {
    return tier_used == LostTier::LOCAL ? local_value : wide_value;
}

/**
 * @brief "越低越好"类指标的置信度映射
 *
 * 线性缩放到 [0, 1]，值为 0 时置信度 1，值达到 threshold 时置信度 0。
 */
auto confidence_lower_better(double value, double threshold) -> double {
    const auto safe_threshold = std::max(1e-6, threshold);
    if (!std::isfinite(value))
        return 0.0;
    return std::clamp(1.0 - value / safe_threshold, 0.0, 1.0);
}

/**
 * @brief "越高越好"类指标的置信度映射
 *
 * 线性缩放到 [0, 1]，值为 threshold 时置信度 1，值为 0 时置信度 0。
 */
auto confidence_higher_better(double value, double threshold) -> double {
    const auto safe_threshold = std::max(1e-6, threshold);
    if (!std::isfinite(value))
        return 0.0;
    return std::clamp(value / safe_threshold, 0.0, 1.0);
}

} // namespace

/**
 * @brief 验证器内部实现
 *
 * 提供初始重定位和丢失重定位两种验证管线，每种管线对位置估计进行多维度检查：
 * 场地边界、配准分数、位置误差、姿态误差，以及丢失重定位特有的内点比例和层级自适应阈值。
 */
struct Validator::Impl {
    /**
     * @brief 构造验证器内部实现，保存初始和丢失两种配置
     */
    Impl(const InitialValidationConfig& initial_config, const LostValidationConfig& lost_config)
        : initial_config_(initial_config)
        , lost_config_(lost_config) {}

    /**
     * @brief 从旋转矩阵提取 yaw 角（弧度）
     */
    static auto yaw_from_rotation(const Eigen::Matrix3f& rotation) -> float {
        const auto yaw = std::atan2(rotation(1, 0), rotation(0, 0));
        return yaw;
    }

    /**
     * @brief 计算两个 yaw 角之间的最短差值（弧度，±π 范围内）
     */
    static auto wrapped_angle_delta(float current, float reference) -> float {
        return static_cast<float>(
            std::atan2(std::sin(current - reference), std::cos(current - reference)));
    }

    /**
     * @brief 检查 3D 位置是否在场地边界立方体内
     */
    static auto
        is_within_field_bounds(const Eigen::Vector3f& position, const FieldBoundsConfig& bounds)
            -> bool {
        return position.x() >= bounds.minimum_x && position.x() <= bounds.maximum_x
            && position.y() >= bounds.minimum_y && position.y() <= bounds.maximum_y
            && position.z() >= bounds.minimum_z && position.z() <= bounds.maximum_z;
    }

    auto evaluate_initial(
        const Eigen::Isometry3f& world_to_base_guess,
        const Eigen::Isometry3f& world_to_base_estimated, double score) const -> ValidationResult;

    auto evaluate_lost(
        const LostPrior& prior, const Eigen::Isometry3f& world_to_base_estimated, double score,
        double inlier_ratio, LostTier tier_used) const -> ValidationResult;

    InitialValidationConfig initial_config_;
    LostValidationConfig lost_config_;
};

/**
 * @brief 评估初始重定位结果
 *
 * 对初始重定位结果进行多维度验证，包括：
 * - 场地边界检查
 * - 配准分数验证
 * - 位置误差检查
 * - 姿态误差检查
 *
 * @param world_to_base_guess 初始猜测位置
 * @param world_to_base_estimated 估计的位置
 * @param score 配准分数
 * @return ValidationResult 验证结果
 */
auto Validator::Impl::evaluate_initial(
    const Eigen::Isometry3f& world_to_base_guess, const Eigen::Isometry3f& world_to_base_estimated,
    double score) const -> ValidationResult {
    // 检查估计位置是否在场地边界内
    const auto estimated_translation = world_to_base_estimated.translation();
    const auto within_bounds =
        is_within_field_bounds(estimated_translation, initial_config_.field_bounds);

    // 计算位置误差和姿态误差
    const auto initial_translation = world_to_base_guess.translation();
    const auto translation_error = (estimated_translation - initial_translation).norm();
    const auto yaw_error = std::abs(wrapped_angle_delta(
        yaw_from_rotation(world_to_base_estimated.rotation()),
        yaw_from_rotation(world_to_base_guess.rotation())));

    // 验证各项指标是否满足阈值要求
    const auto score_ok = std::isfinite(score) && score <= initial_config_.score_threshold;
    const auto distance_ok = translation_error <= initial_config_.initial_max_translation_error_m;
    const auto yaw_ok = yaw_error <= static_cast<float>(
                            initial_config_.initial_max_yaw_error_deg * std::numbers::pi / 180.0);

    // 综合判断是否接受该重定位结果
    const auto accepted = within_bounds && score_ok && distance_ok && yaw_ok;

    return ValidationResult{
        .within_bounds = within_bounds,
        .score_ok = score_ok,
        .inlier_ok = true,                    // 初始重定位不检查内点比例
        .distance_ok = distance_ok,
        .yaw_ok = yaw_ok,
        .accepted = accepted,
        .confidence = accepted ? 1.0F : 0.0F, // 初始重定位使用二元置信度
    };
}

/**
 * @brief 评估丢失重定位结果
 *
 * 对丢失重定位结果进行多层级验证，支持LOCAL和WIDE两种模式。
 * 使用加权置信度计算，提供更细粒度的质量评估。
 *
 * @param prior 先验信息（可能包含最后已知位置）
 * @param world_to_base_estimated 估计的位置
 * @param score 配准分数
 * @param inlier_ratio 内点比例
 * @param tier_used 使用的重定位层级
 * @return ValidationResult 验证结果
 */
auto Validator::Impl::evaluate_lost(
    const LostPrior& prior, const Eigen::Isometry3f& world_to_base_estimated, double score,
    double inlier_ratio, LostTier tier_used) const -> ValidationResult {
    // 检查估计位置是否在场地边界内
    const auto estimated_translation = world_to_base_estimated.translation();
    const auto within_bounds =
        is_within_field_bounds(estimated_translation, lost_config_.field_bounds);

    /**
     * 根据重定位层级选择验证阈值
     * LOCAL模式使用更严格的阈值，WIDE模式使用更宽松的阈值
     */
    const auto score_threshold = select_by_tier(
        tier_used, lost_config_.score_threshold_local, lost_config_.score_threshold_wide);
    const auto min_inlier_ratio = select_by_tier(
        tier_used, lost_config_.min_inlier_ratio_local, lost_config_.min_inlier_ratio_wide);
    const auto max_distance_from_prior = select_by_tier(
        tier_used, lost_config_.max_distance_from_prior_local_m,
        lost_config_.max_distance_from_prior_wide_m);
    const auto max_yaw_from_prior_rad = select_by_tier(
                                            tier_used, lost_config_.max_yaw_from_prior_local_deg,
                                            lost_config_.max_yaw_from_prior_wide_deg)
                                      * std::numbers::pi / 180.0;

    // 验证配准分数和内点比例
    const auto score_ok = std::isfinite(score) && score <= score_threshold;
    const auto inlier_ok = std::isfinite(inlier_ratio) && inlier_ratio >= min_inlier_ratio;

    // 初始化距离和姿态误差检查
    auto distance_from_prior = 0.0;
    auto yaw_error = 0.0;
    auto distance_ok = true;
    auto yaw_ok = true;

    // 如果有先验信息，检查与先验位置的偏差
    if (prior.has_prior) {
        distance_from_prior =
            (world_to_base_estimated.translation() - prior.world_to_base.translation()).norm();
        yaw_error = std::abs(wrapped_angle_delta(
            yaw_from_rotation(world_to_base_estimated.rotation()),
            yaw_from_rotation(prior.world_to_base.rotation())));

        distance_ok = distance_from_prior <= max_distance_from_prior;
        yaw_ok = yaw_error <= max_yaw_from_prior_rad;
    }

    // 综合判断是否接受该重定位结果
    const auto accepted = within_bounds && score_ok && inlier_ok && distance_ok && yaw_ok;

    const auto score_term = confidence_lower_better(score, score_threshold);
    const auto inlier_term = confidence_higher_better(inlier_ratio, min_inlier_ratio);
    const auto distance_term =
        prior.has_prior ? confidence_lower_better(distance_from_prior, max_distance_from_prior)
                        : 1.0;
    const auto yaw_term =
        prior.has_prior ? confidence_lower_better(yaw_error, max_yaw_from_prior_rad) : 1.0;

    /**
     * @brief 计算加权置信度（0-1），权重分配如下：
     *
     * 魔法参数
     */
    const auto confidence = std::clamp(
        0.4 * score_term + 0.3 * inlier_term + 0.2 * distance_term + 0.1 * yaw_term, 0.0, 1.0);

    return ValidationResult{
        .within_bounds = within_bounds,
        .score_ok = score_ok,
        .inlier_ok = inlier_ok,
        .distance_ok = distance_ok,
        .yaw_ok = yaw_ok,
        .accepted = accepted,
        .confidence = static_cast<float>(confidence),
    };
}

Validator::Validator(
    const InitialValidationConfig& initial_config, const LostValidationConfig& lost_config)
    : pimpl_(std::make_unique<Impl>(initial_config, lost_config)) {}

Validator::~Validator() = default;

auto Validator::evaluate_initial(
    const Eigen::Isometry3f& world_to_base_guess, const Eigen::Isometry3f& world_to_base_estimated,
    double score) const -> ValidationResult {
    return pimpl_->evaluate_initial(world_to_base_guess, world_to_base_estimated, score);
}

auto Validator::evaluate_lost(
    const LostPrior& prior, const Eigen::Isometry3f& world_to_base_estimated, double score,
    double inlier_ratio, LostTier tier_used) const -> ValidationResult {
    return pimpl_->evaluate_lost(prior, world_to_base_estimated, score, inlier_ratio, tier_used);
}

} // namespace rmcs::location

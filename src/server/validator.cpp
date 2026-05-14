#include "validator.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace rmcs::location {

namespace {

auto confidence_lower_better(double value, double threshold) -> double {
    const auto safe_threshold = std::max(1e-6, threshold);
    if (!std::isfinite(value))
        return 0.0;
    return std::clamp(1.0 - value / safe_threshold, 0.0, 1.0);
}

auto confidence_higher_better(double value, double threshold) -> double {
    const auto safe_threshold = std::max(1e-6, threshold);
    if (!std::isfinite(value))
        return 0.0;
    return std::clamp(value / safe_threshold, 0.0, 1.0);
}

auto yaw_from_rotation(const Eigen::Matrix3f& rotation) -> float {
    return std::atan2(rotation(1, 0), rotation(0, 0));
}

auto wrapped_angle_delta(float current, float reference) -> float {
    return static_cast<float>(
        std::atan2(std::sin(current - reference), std::cos(current - reference)));
}

auto is_within_field_bounds(const Eigen::Vector3f& position, const FieldBoundsConfig& bounds)
    -> bool {
    return position.x() >= bounds.minimum_x && position.x() <= bounds.maximum_x
        && position.y() >= bounds.minimum_y && position.y() <= bounds.maximum_y
        && position.z() >= bounds.minimum_z && position.z() <= bounds.maximum_z;
}

/// LOCAL/WIDE 共用的 prior-relative 验收 + confidence 计算
auto evaluate_with_prior_thresholds(
    const RegistrationPrior& prior, const Eigen::Isometry3f& world_to_base_estimated, double score,
    double inlier_ratio, const FieldBoundsConfig& field_bounds, double score_threshold,
    double min_inlier_ratio, double max_distance_from_prior_m, double max_yaw_from_prior_deg)
    -> ValidationResult {
    const auto estimated_translation = world_to_base_estimated.translation();
    const auto within_bounds = is_within_field_bounds(estimated_translation, field_bounds);

    const auto max_yaw_from_prior_rad = max_yaw_from_prior_deg * std::numbers::pi / 180.0;

    const auto score_ok = std::isfinite(score) && score <= score_threshold;
    const auto inlier_ok = std::isfinite(inlier_ratio) && inlier_ratio >= min_inlier_ratio;

    auto distance_from_prior = 0.0;
    auto yaw_error = 0.0;
    auto distance_ok = true;
    auto yaw_ok = true;

    if (prior.has_prior) {
        distance_from_prior = (estimated_translation - prior.world_to_base.translation()).norm();
        yaw_error = std::abs(wrapped_angle_delta(
            yaw_from_rotation(world_to_base_estimated.rotation()),
            yaw_from_rotation(prior.world_to_base.rotation())));

        distance_ok = distance_from_prior <= max_distance_from_prior_m;
        yaw_ok = yaw_error <= max_yaw_from_prior_rad;
    }

    const auto accepted = within_bounds && score_ok && inlier_ok && distance_ok && yaw_ok;

    const auto score_term = confidence_lower_better(score, score_threshold);
    const auto inlier_term = confidence_higher_better(inlier_ratio, min_inlier_ratio);
    const auto distance_term =
        prior.has_prior ? confidence_lower_better(distance_from_prior, max_distance_from_prior_m)
                        : 1.0;
    const auto yaw_term =
        prior.has_prior ? confidence_lower_better(yaw_error, max_yaw_from_prior_rad) : 1.0;

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

} // namespace

struct Validator::Impl {
    Impl(
        const InitialValidationConfig& initial_config, const LocalValidationConfig& local_config,
        const WideValidationConfig& wide_config)
        : initial_config_(initial_config)
        , local_config_(local_config)
        , wide_config_(wide_config) {}

    auto evaluate_initial(
        const Eigen::Isometry3f& world_to_base_guess,
        const Eigen::Isometry3f& world_to_base_estimated, double score) const -> ValidationResult;

    auto evaluate_local(
        const RegistrationPrior& prior, const Eigen::Isometry3f& world_to_base_estimated,
        double score, double inlier_ratio) const -> ValidationResult;

    auto evaluate_wide(
        const Eigen::Isometry3f& world_to_base_estimated, double score, double inlier_ratio) const
        -> ValidationResult;

    InitialValidationConfig initial_config_;
    LocalValidationConfig local_config_;
    WideValidationConfig wide_config_;
};

auto Validator::Impl::evaluate_initial(
    const Eigen::Isometry3f& world_to_base_guess, const Eigen::Isometry3f& world_to_base_estimated,
    double score) const -> ValidationResult {
    const auto estimated_translation = world_to_base_estimated.translation();
    const auto within_bounds =
        is_within_field_bounds(estimated_translation, initial_config_.field_bounds);

    const auto initial_translation = world_to_base_guess.translation();
    const auto translation_error = (estimated_translation - initial_translation).norm();
    const auto yaw_error = std::abs(wrapped_angle_delta(
        yaw_from_rotation(world_to_base_estimated.rotation()),
        yaw_from_rotation(world_to_base_guess.rotation())));

    const auto score_ok = std::isfinite(score) && score <= initial_config_.score_threshold;
    const auto distance_ok = translation_error <= initial_config_.initial_max_translation_error_m;
    const auto yaw_ok = yaw_error <= static_cast<float>(
                            initial_config_.initial_max_yaw_error_deg * std::numbers::pi / 180.0);

    const auto accepted = within_bounds && score_ok && distance_ok && yaw_ok;

    return ValidationResult{
        .within_bounds = within_bounds,
        .score_ok = score_ok,
        .inlier_ok = true,
        .distance_ok = distance_ok,
        .yaw_ok = yaw_ok,
        .accepted = accepted,
        .confidence = accepted ? 1.0F : 0.0F,
    };
}

auto Validator::Impl::evaluate_local(
    const RegistrationPrior& prior, const Eigen::Isometry3f& world_to_base_estimated, double score,
    double inlier_ratio) const -> ValidationResult {
    return evaluate_with_prior_thresholds(
        prior, world_to_base_estimated, score, inlier_ratio, local_config_.field_bounds,
        local_config_.score_threshold, local_config_.min_inlier_ratio,
        local_config_.max_distance_from_prior_m, local_config_.max_yaw_from_prior_deg);
}

auto Validator::Impl::evaluate_wide(
    const Eigen::Isometry3f& world_to_base_estimated, double score, double inlier_ratio) const
    -> ValidationResult {
    return evaluate_with_prior_thresholds(
        RegistrationPrior{}, world_to_base_estimated, score, inlier_ratio,
        wide_config_.field_bounds, wide_config_.score_threshold, wide_config_.min_inlier_ratio, 1.0,
        1.0);
}

Validator::Validator(
    const InitialValidationConfig& initial_config, const LocalValidationConfig& local_config,
    const WideValidationConfig& wide_config)
    : pimpl_(std::make_unique<Impl>(initial_config, local_config, wide_config)) {}

Validator::~Validator() = default;

auto Validator::evaluate_initial(
    const Eigen::Isometry3f& world_to_base_guess, const Eigen::Isometry3f& world_to_base_estimated,
    double score) const -> ValidationResult {
    return pimpl_->evaluate_initial(world_to_base_guess, world_to_base_estimated, score);
}

auto Validator::evaluate_local(
    const RegistrationPrior& prior, const Eigen::Isometry3f& world_to_base_estimated, double score,
    double inlier_ratio) const -> ValidationResult {
    return pimpl_->evaluate_local(prior, world_to_base_estimated, score, inlier_ratio);
}

auto Validator::evaluate_wide(
    const Eigen::Isometry3f& world_to_base_estimated, double score, double inlier_ratio) const
    -> ValidationResult {
    return pimpl_->evaluate_wide(world_to_base_estimated, score, inlier_ratio);
}

} // namespace rmcs::location

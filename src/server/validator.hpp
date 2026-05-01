#pragma once

#include <cstdint>

#include <Eigen/Geometry>

#include "common/pimpl.hpp"
#include "tools/registration_tools.hpp"

namespace rmcs::location {

struct FieldBoundsConfig {
    double minimum_x = -5.0;
    double maximum_x = 25.0;
    double minimum_y = -8.0;
    double maximum_y = 8.0;
    double minimum_z = -0.5;
    double maximum_z = 1.0;
};

struct InitialValidationConfig {
    double score_threshold = 0.01;
    double initial_max_translation_error_m = 2.0;
    double initial_max_yaw_error_deg = 30.0;

    FieldBoundsConfig field_bounds {};
};

struct LostValidationConfig {
    FieldBoundsConfig field_bounds {};

    double score_threshold_local = 0.015;
    double score_threshold_wide = 0.03;

    double min_inlier_ratio_local = 0.35;
    double min_inlier_ratio_wide = 0.25;

    double max_distance_from_prior_local_m = 1.5;
    double max_distance_from_prior_wide_m = 5.0;

    double max_yaw_from_prior_local_deg = 20.0;
    double max_yaw_from_prior_wide_deg = 60.0;
};

struct ValidationResult {
    bool within_bounds = false;
    bool score_ok = false;
    bool inlier_ok = false;
    bool distance_ok = false;
    bool yaw_ok = false;
    bool accepted = false;

    float confidence = 0.0F;
};

class Validator final {
public:
    Validator(
        const InitialValidationConfig& initial_config,
        const LostValidationConfig& lost_config);
    ~Validator();

    RMCS_LOCATION_DELETE_COPY(Validator)

    auto evaluate_initial(
        const Eigen::Isometry3f& world_to_base_guess,
        const Eigen::Isometry3f& world_to_base_estimated,
        double score) const -> ValidationResult;

    auto evaluate_lost(
        const LostPrior& prior,
        const Eigen::Isometry3f& world_to_base_estimated,
        double score,
        double inlier_ratio,
        LostTier tier_used) const -> ValidationResult;

private:
    RMCS_LOCATION_DECLARE_PIMPL(Validator)
};

} // namespace rmcs::location

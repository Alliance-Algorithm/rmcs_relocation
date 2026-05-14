#include <cmath>
#include <numbers>

#include <gtest/gtest.h>

#include "server/validator.hpp"

using namespace rmcs::location;

namespace {

auto make_identity_pose() -> Eigen::Isometry3f { return Eigen::Isometry3f::Identity(); }

auto make_pose(float x, float y, float z, float yaw_deg) -> Eigen::Isometry3f {
    const auto yaw_rad = yaw_deg * std::numbers::pi_v<float> / 180.0F;
    auto pose = Eigen::Isometry3f::Identity();
    pose.translation() << x, y, z;
    pose.rotate(Eigen::AngleAxisf{yaw_rad, Eigen::Vector3f::UnitZ()});
    return pose;
}

auto make_initial_config() -> InitialValidationConfig {
    auto config = InitialValidationConfig{};
    config.score_threshold = 0.04;
    config.initial_max_translation_error_m = 0.5;
    config.initial_max_yaw_error_deg = 30.0;
    config.field_bounds = FieldBoundsConfig{-2.0, 7.0, -5.5, 5.5, -0.5, 1.0};
    return config;
}

auto make_local_config() -> LocalValidationConfig {
    auto config = LocalValidationConfig{};
    config.field_bounds = FieldBoundsConfig{-2.0, 7.0, -5.5, 5.5, -0.5, 1.0};
    config.score_threshold = 0.08;
    config.min_inlier_ratio = 0.20;
    config.max_distance_from_prior_m = 3.0;
    config.max_yaw_from_prior_deg = 45.0;
    return config;
}

auto make_wide_config() -> WideValidationConfig {
    auto config = WideValidationConfig{};
    config.field_bounds = FieldBoundsConfig{-2.0, 7.0, -5.5, 5.5, -0.5, 1.0};
    config.score_threshold = 0.08;
    config.min_inlier_ratio = 0.15;
    return config;
}

auto make_validator() -> Validator {
    return Validator{make_initial_config(), make_local_config(), make_wide_config()};
}

} // namespace

TEST(ValidatorTest, InitialAcceptedWhenAllPass) {
    auto validator = make_validator();
    const auto guess = make_pose(0.0F, 0.0F, 0.0F, 0.0F);
    const auto estimated = make_pose(0.2F, 0.1F, 0.0F, 5.0F);

    const auto result = validator.evaluate_initial(guess, estimated, 0.02);
    EXPECT_TRUE(result.accepted);
    EXPECT_TRUE(result.score_ok);
    EXPECT_TRUE(result.distance_ok);
    EXPECT_TRUE(result.yaw_ok);
    EXPECT_TRUE(result.within_bounds);
}

TEST(ValidatorTest, InitialRejectedWhenScoreTooHigh) {
    auto validator = make_validator();
    const auto guess = make_identity_pose();
    const auto estimated = make_identity_pose();

    const auto result = validator.evaluate_initial(guess, estimated, 0.10);
    EXPECT_FALSE(result.accepted);
    EXPECT_FALSE(result.score_ok);
}

TEST(ValidatorTest, InitialRejectedWhenTranslationTooLarge) {
    auto validator = make_validator();
    const auto guess = make_identity_pose();
    const auto estimated = make_pose(1.0F, 0.0F, 0.0F, 0.0F);

    const auto result = validator.evaluate_initial(guess, estimated, 0.02);
    EXPECT_FALSE(result.accepted);
    EXPECT_FALSE(result.distance_ok);
}

TEST(ValidatorTest, InitialRejectedWhenYawTooLarge) {
    auto validator = make_validator();
    const auto guess = make_identity_pose();
    const auto estimated = make_pose(0.0F, 0.0F, 0.0F, 45.0F);

    const auto result = validator.evaluate_initial(guess, estimated, 0.02);
    EXPECT_FALSE(result.accepted);
    EXPECT_FALSE(result.yaw_ok);
}

TEST(ValidatorTest, InitialRejectedWhenOutOfBounds) {
    auto validator = make_validator();
    const auto guess = make_pose(10.0F, 10.0F, 0.0F, 0.0F);
    const auto estimated = guess;

    const auto result = validator.evaluate_initial(guess, estimated, 0.02);
    EXPECT_FALSE(result.accepted);
    EXPECT_FALSE(result.within_bounds);
}

TEST(ValidatorTest, LocalAcceptedWhenAllPass) {
    auto validator = make_validator();

    auto prior = RegistrationPrior{};
    prior.has_prior = true;
    prior.world_to_base = make_pose(2.0F, 3.0F, 0.0F, 0.0F);

    const auto estimated = make_pose(2.5F, 3.2F, 0.0F, 10.0F);
    const auto result = validator.evaluate_local(prior, estimated, 0.05, 0.30);
    EXPECT_TRUE(result.accepted);
}

TEST(ValidatorTest, WideRejectedWhenInlierTooLow) {
    auto validator = make_validator();

    const auto estimated = make_pose(2.1F, 3.1F, 0.0F, 5.0F);
    const auto result = validator.evaluate_wide(estimated, 0.05, 0.10);
    EXPECT_FALSE(result.accepted);
    EXPECT_FALSE(result.inlier_ok);
}

TEST(ValidatorTest, ConfidenceInRange) {
    auto validator = make_validator();

    const auto initial_result =
        validator.evaluate_initial(make_identity_pose(), make_identity_pose(), 0.01);
    EXPECT_GE(initial_result.confidence, 0.0F);
    EXPECT_LE(initial_result.confidence, 1.0F);

    auto prior = RegistrationPrior{};
    prior.has_prior = true;
    prior.world_to_base = make_identity_pose();
    const auto local_result = validator.evaluate_local(prior, make_identity_pose(), 0.01, 0.50);
    EXPECT_GE(local_result.confidence, 0.0F);
    EXPECT_LE(local_result.confidence, 1.0F);

    const auto wide_result = validator.evaluate_wide(make_identity_pose(), 0.01, 0.50);
    EXPECT_GE(wide_result.confidence, 0.0F);
    EXPECT_LE(wide_result.confidence, 1.0F);
}

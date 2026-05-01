#include "tools/param_tools.hpp"

#include <string>
#include <string_view>

#include "tools/geometry_tools.hpp"
#include "tools/numeric_tools.hpp"

namespace rmcs::location::tools {

ParamReader::ParamReader(rclcpp::Node& node)
    : node_(node) {}

auto ParamReader::read_string(std::string_view key, std::string_view default_value) const
    -> std::string {
    return node_.declare_parameter<std::string>(std::string(key), std::string(default_value));
}

auto ParamReader::read_double(std::string_view key, double default_value) const -> double {
    return node_.declare_parameter<double>(std::string(key), default_value);
}

auto ParamReader::read_int(std::string_view key, int default_value) const -> int {
    return node_.declare_parameter<int>(std::string(key), default_value);
}

auto ParamReader::read_bool(std::string_view key, bool default_value) const -> bool {
    return node_.declare_parameter<bool>(std::string(key), default_value);
}

auto ParamReader::read_positive_int(std::string_view key, int default_value) const -> int {
    return sanitize_positive_int(read_int(key, default_value), default_value);
}

auto ParamReader::read_bounds(std::string_view prefix, const FieldBoundsConfig& defaults) const
    -> FieldBoundsConfig {
    const auto key_prefix = std::string(prefix);
    auto bounds = FieldBoundsConfig {};
    bounds.minimum_x = read_double(key_prefix + ".min_x", defaults.minimum_x);
    bounds.maximum_x = read_double(key_prefix + ".max_x", defaults.maximum_x);
    bounds.minimum_y = read_double(key_prefix + ".min_y", defaults.minimum_y);
    bounds.maximum_y = read_double(key_prefix + ".max_y", defaults.maximum_y);
    bounds.minimum_z = read_double(key_prefix + ".min_z", defaults.minimum_z);
    bounds.maximum_z = read_double(key_prefix + ".max_z", defaults.maximum_z);
    return bounds;
}

auto ParamReader::read_pose(std::string_view prefix, const geometry_msgs::msg::Pose& defaults) const
    -> geometry_msgs::msg::Pose {
    const auto key_prefix = std::string(prefix);
    auto pose = geometry_msgs::msg::Pose {};
    pose.position.x = read_double(key_prefix + ".translation.x", defaults.position.x);
    pose.position.y = read_double(key_prefix + ".translation.y", defaults.position.y);
    pose.position.z = read_double(key_prefix + ".translation.z", defaults.position.z);

    pose.orientation.w = read_double(key_prefix + ".orientation.w", defaults.orientation.w);
    pose.orientation.x = read_double(key_prefix + ".orientation.x", defaults.orientation.x);
    pose.orientation.y = read_double(key_prefix + ".orientation.y", defaults.orientation.y);
    pose.orientation.z = read_double(key_prefix + ".orientation.z", defaults.orientation.z);

    return normalize_pose(pose);
}

auto read_pose_parameter(
    rclcpp::Node& node, const std::string& prefix, const geometry_msgs::msg::Pose& defaults)
    -> geometry_msgs::msg::Pose {
    auto normalized_defaults = defaults;
    if (normalized_defaults.orientation.w == 0.0 && normalized_defaults.orientation.x == 0.0
        && normalized_defaults.orientation.y == 0.0 && normalized_defaults.orientation.z == 0.0) {
        normalized_defaults.orientation.w = 1.0;
    }

    auto reader = ParamReader { node };
    return reader.read_pose(prefix, normalized_defaults);
}

namespace {

auto load_initial_runtime_config(ParamReader& reader) -> InitialRuntimeConfig {
    auto config = InitialRuntimeConfig {};
    config.pointcloud_topic =
        reader.read_string("initial.pointcloud_topic", "/cloud_registered_undistort");
    config.collect_duration_sec =
        reader.read_double("initial.collect_duration_sec", 2.0);
    config.min_accumulated_points =
        reader.read_int("initial.min_accumulated_points", 2500);
    config.submap_radius_m =
        reader.read_double("initial.submap_radius_m", 25.0);
    return config;
}

auto load_initial_registration_config(ParamReader& reader) -> InitialRegistrationConfig {
    auto config = InitialRegistrationConfig {};
    config.coarse_iterations =
        reader.read_int("initial.coarse_iterations", 50);
    config.refine_iterations =
        reader.read_int("initial.refine_iterations", 20);
    config.precise_iterations =
        reader.read_int("initial.precise_iterations", 500);
    config.max_correspondence_distance_m =
        reader.read_double("initial.max_correspondence_distance_m", 5.0);
    config.score_threshold =
        reader.read_double("initial.score_threshold", 0.01);
    config.yaw_search_window_deg =
        reader.read_double("initial.yaw_search_window_deg", 30.0);
    config.coarse_yaw_step_deg =
        reader.read_double("initial.coarse_yaw_step_deg", 15.0);
    config.refine_yaw_step_deg =
        reader.read_double("initial.refine_yaw_step_deg", 5.0);
    config.coarse_top_k =
        static_cast<std::size_t>(reader.read_positive_int("initial.coarse_top_k", 2));
    config.voxel_leaf_m =
        reader.read_double("initial.voxel_leaf_m", 0.2);
    config.outlier_mean_k =
        reader.read_int("initial.outlier_mean_k", 50);
    config.outlier_stddev_mul_thresh =
        reader.read_double("initial.outlier_stddev_mul_thresh", 0.5);
    return config;
}

auto load_initial_validation_config(
    ParamReader& reader, const InitialRegistrationConfig& initial_registration_config)
    -> InitialValidationConfig {
    auto config = InitialValidationConfig {};
    config.score_threshold = initial_registration_config.score_threshold;
    config.initial_max_translation_error_m =
        reader.read_double("initial.initial_max_translation_error_m", 2.0);
    config.initial_max_yaw_error_deg =
        reader.read_double("initial.initial_max_yaw_error_deg", 30.0);
    config.field_bounds = reader.read_bounds(
        "initial.field_bounds",
        FieldBoundsConfig {});
    return config;
}

auto load_lost_runtime_config(ParamReader& reader, const InitialRuntimeConfig& initial_runtime_config)
    -> LostRuntimeConfig {
    auto config = LostRuntimeConfig {};
    config.pointcloud_topic =
        reader.read_string("lost.pointcloud_topic", initial_runtime_config.pointcloud_topic);
    config.collect_duration_sec =
        reader.read_double("lost.collect_duration_sec", 2.0);
    config.min_accumulated_points =
        reader.read_int("lost.min_accumulated_points", 2500);
    return config;
}

auto load_lost_registration_config(ParamReader& reader) -> LostRegistrationConfig {
    auto config = LostRegistrationConfig {};
    config.max_candidate_count =
        static_cast<std::size_t>(reader.read_positive_int("lost.max_candidate_count", 3));
    config.local_sigma_xy_m =
        reader.read_double("lost.local_sigma_xy_m", 1.0);
    config.local_sigma_yaw_deg =
        reader.read_double("lost.local_sigma_yaw_deg", 15.0);

    config.submap_radius_local_m =
        reader.read_double("lost.submap_radius_local_m", 10.0);
    config.submap_radius_wide_m =
        reader.read_double("lost.submap_radius_wide_m", 18.0);

    config.coarse_iterations_local =
        reader.read_int("lost.coarse_iterations_local", 30);
    config.refine_iterations_local =
        reader.read_int("lost.refine_iterations_local", 20);
    config.precise_iterations_local =
        reader.read_int("lost.precise_iterations_local", 200);

    config.coarse_iterations_wide =
        reader.read_int("lost.coarse_iterations_wide", 40);
    config.refine_iterations_wide =
        reader.read_int("lost.refine_iterations_wide", 20);
    config.precise_iterations_wide =
        reader.read_int("lost.precise_iterations_wide", 300);

    config.max_correspondence_distance_m =
        reader.read_double("lost.max_correspondence_distance_m", 2.5);
    config.coarse_score_threshold_local =
        reader.read_double("lost.coarse_score_threshold_local", 0.3);
    config.coarse_score_threshold_wide =
        reader.read_double("lost.coarse_score_threshold_wide", 0.15);
    config.local_yaw_window_deg =
        reader.read_double("lost.local_yaw_window_deg", 20.0);
    config.wide_yaw_window_deg =
        reader.read_double("lost.wide_yaw_window_deg", 60.0);
    config.local_coarse_yaw_step_deg =
        reader.read_double("lost.local_coarse_yaw_step_deg", 10.0);
    config.wide_coarse_yaw_step_deg =
        reader.read_double("lost.wide_coarse_yaw_step_deg", 15.0);
    config.refine_yaw_step_deg =
        reader.read_double("lost.refine_yaw_step_deg", 5.0);

    config.enable_map_consistency_filter =
        reader.read_bool("lost.enable_map_consistency_filter", true);
    config.map_consistency_distance_m =
        reader.read_double("lost.map_consistency_distance_m", 0.5);
    config.min_retained_fraction =
        reader.read_double("lost.min_retained_fraction", 0.25);

    config.rank_weight_inlier =
        reader.read_double("lost.rank_weight_inlier", 0.5);
    config.rank_weight_distance =
        reader.read_double("lost.rank_weight_distance", 0.3);
    return config;
}

auto load_lost_validation_config(
    ParamReader& reader,
    const InitialValidationConfig& initial_validation_config,
    LostRegistrationConfig& lost_registration_config) -> LostValidationConfig {
    const auto max_distance_from_prior_local_m =
        reader.read_double("lost.max_distance_from_prior_local_m", 1.5);
    const auto max_distance_from_prior_wide_m =
        reader.read_double("lost.max_distance_from_prior_wide_m", 5.0);

    lost_registration_config.max_distance_from_prior_local_m = max_distance_from_prior_local_m;
    lost_registration_config.max_distance_from_prior_wide_m = max_distance_from_prior_wide_m;

    auto config = LostValidationConfig {};
    config.field_bounds = reader.read_bounds(
        "lost.field_bounds",
        initial_validation_config.field_bounds);
    config.score_threshold_local =
        reader.read_double("lost.score_threshold_local", 0.015);
    config.score_threshold_wide =
        reader.read_double("lost.score_threshold_wide", 0.03);
    lost_registration_config.score_threshold_wide = config.score_threshold_wide;
    config.min_inlier_ratio_local =
        reader.read_double("lost.min_inlier_ratio_local", 0.35);
    config.min_inlier_ratio_wide =
        reader.read_double("lost.min_inlier_ratio_wide", 0.25);
    config.max_distance_from_prior_local_m =
        max_distance_from_prior_local_m;
    config.max_distance_from_prior_wide_m =
        max_distance_from_prior_wide_m;
    config.max_yaw_from_prior_local_deg =
        reader.read_double("lost.max_yaw_from_prior_local_deg", 20.0);
    config.max_yaw_from_prior_wide_deg =
        reader.read_double("lost.max_yaw_from_prior_wide_deg", 60.0);
    return config;
}

auto load_health_runtime_config(ParamReader& reader) -> HealthRuntimeConfig {
    auto config = HealthRuntimeConfig {};
    config.rate_hz = reader.read_double("health.rate_hz", 5.0);
    config.sample_points =
        reader.read_positive_int("health.sample_points", 500);
    config.warn_threshold_m =
        reader.read_double("health.warn_threshold_m", 0.25);
    config.lost_threshold_m =
        reader.read_double("health.lost_threshold_m", 0.45);
    config.min_inlier_ratio =
        reader.read_double("health.min_inlier_ratio", 0.30);
    config.warn_dwell_sec =
        reader.read_double("health.warn_dwell_sec", 0.6);
    config.lost_dwell_sec =
        reader.read_double("health.lost_dwell_sec", 1.0);
    config.recover_margin_m =
        reader.read_double("health.recover_margin_m", 0.05);
    config.recover_dwell_sec =
        reader.read_double("health.recover_dwell_sec", 2.0);
    config.inlier_distance_m =
        reader.read_double("health.inlier_distance_m", 0.5);
    return config;
}

} // namespace

auto load_runtime_params(rclcpp::Node& node) -> RuntimeParamsBundle {
    auto reader = ParamReader { node };
    auto params = RuntimeParamsBundle {};

    params.map_path = reader.read_string("map_path", "");
    params.service_name = reader.read_string("service_name", "/rmcs_relocation/relocalize");
    params.world_frame = reader.read_string("world_frame", "world");
    params.odom_frame = reader.read_string("odom_frame", "odom");
    params.base_frame = reader.read_string("base_frame", "base_link");
    params.publish_tf_rate_hz = reader.read_double("publish_tf_rate_hz", 10.0);

    params.initial_runtime_config = load_initial_runtime_config(reader);
    params.initial_registration_config = load_initial_registration_config(reader);
    params.initial_validation_config =
        load_initial_validation_config(reader, params.initial_registration_config);
    params.lost_runtime_config = load_lost_runtime_config(reader, params.initial_runtime_config);
    params.lost_registration_config = load_lost_registration_config(reader);
    params.lost_validation_config = load_lost_validation_config(
        reader, params.initial_validation_config, params.lost_registration_config);
    params.health_runtime_config = load_health_runtime_config(reader);

    return params;
}

} // namespace rmcs::location::tools

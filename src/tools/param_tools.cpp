#include "tools/param_tools.hpp"

#include <string>
#include <string_view>

#include "tools/geometry_tools.hpp"
#include "tools/numeric_tools.hpp"

namespace rmcs::location::tools {

ParamReader::ParamReader(rclcpp::Node& node)
    : node_(node) {}

namespace {

template <typename T>
auto read_or_declare_parameter(rclcpp::Node& node, std::string_view key, const T& default_value)
    -> T {
    const auto name = std::string(key);
    if (node.has_parameter(name)) {
        auto value = T{};
        if (node.get_parameter(name, value))
            return value;
        return default_value;
    }
    return node.declare_parameter<T>(name, default_value);
}

} // namespace

auto ParamReader::read_string(std::string_view key, std::string_view default_value) const
    -> std::string {
    return read_or_declare_parameter<std::string>(node_, key, std::string(default_value));
}

auto ParamReader::read_double(std::string_view key, double default_value) const -> double {
    return read_or_declare_parameter<double>(node_, key, default_value);
}

auto ParamReader::read_int(std::string_view key, int default_value) const -> int {
    return read_or_declare_parameter<int>(node_, key, default_value);
}

auto ParamReader::read_bool(std::string_view key, bool default_value) const -> bool {
    return read_or_declare_parameter<bool>(node_, key, default_value);
}

auto ParamReader::read_positive_int(std::string_view key, int default_value) const -> int {
    return sanitize_positive_int(read_int(key, default_value), default_value);
}

auto ParamReader::read_bounds(std::string_view prefix, const FieldBoundsConfig& defaults) const
    -> FieldBoundsConfig {
    const auto key_prefix = std::string(prefix);
    auto bounds = FieldBoundsConfig{};
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
    auto pose = geometry_msgs::msg::Pose{};
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

    auto reader = ParamReader{node};
    return reader.read_pose(prefix, normalized_defaults);
}

namespace {

auto load_initial_runtime_config(ParamReader& reader, const std::string& pointcloud_topic)
    -> InitialRuntimeConfig {
    auto config = InitialRuntimeConfig{};
    config.pointcloud_topic = pointcloud_topic;
    config.collect_duration_sec = reader.read_double("initial.collect_duration_sec", 2.0);
    config.min_accumulated_points = reader.read_int("initial.min_accumulated_points", 2500);
    return config;
}

auto load_preprocess_config(ParamReader& reader) -> PointCloudPreprocessConfig {
    auto config = PointCloudPreprocessConfig{};
    config.voxel_leaf_m = reader.read_double("preprocess.voxel_leaf_m", 0.2);
    config.outlier_mean_k = reader.read_int("preprocess.outlier_mean_k", 30);
    config.outlier_stddev_mul_thresh =
        reader.read_double("preprocess.outlier_stddev_mul_thresh", 0.5);
    return config;
}

auto load_gicp_config(ParamReader& reader) -> GicpConfig {
    auto config = GicpConfig{};
    config.num_threads = reader.read_positive_int("gicp.num_threads", 4);
    config.num_neighbors_for_covariance =
        reader.read_positive_int("gicp.num_neighbors_for_covariance", 20);
    config.rotation_epsilon = reader.read_double("gicp.rotation_epsilon", 2e-3);
    config.voxel_resolution = reader.read_double("gicp.voxel_resolution", 0.2);
    config.coarse_transformation_epsilon =
        reader.read_double("gicp.coarse_transformation_epsilon", 1e-3);
    config.precise_transformation_epsilon =
        reader.read_double("gicp.precise_transformation_epsilon", 1e-4);
    return config;
}

auto load_common_registration_config(ParamReader& reader) -> CommonRegistrationConfig {
    return CommonRegistrationConfig{
        .preprocess = load_preprocess_config(reader),
        .gicp = load_gicp_config(reader),
    };
}

auto load_initial_registration_config(ParamReader& reader) -> InitialRegistrationConfig {
    auto config = InitialRegistrationConfig{};
    config.coarse_iterations = reader.read_int("initial.coarse_iterations", 12);
    config.precise_iterations = reader.read_int("initial.precise_iterations", 20);
    config.max_correspondence_distance_m =
        reader.read_double("initial.max_correspondence_distance_m", 0.5);
    config.score_threshold = reader.read_double("initial.score_threshold", 0.04);
    config.yaw_search_window_deg = reader.read_double("initial.yaw_search_window_deg", 15.0);
    config.coarse_yaw_step_deg = reader.read_double("initial.coarse_yaw_step_deg", 15.0);
    config.coarse_top_k =
        static_cast<std::size_t>(reader.read_positive_int("initial.coarse_top_k", 1));
    return config;
}

auto load_initial_validation_config(
    ParamReader& reader, const InitialRegistrationConfig& initial_registration_config,
    const FieldBoundsConfig& field_bounds)
    -> InitialValidationConfig {
    auto config = InitialValidationConfig{};
    config.score_threshold = initial_registration_config.score_threshold;
    config.initial_max_translation_error_m =
        reader.read_double("initial.initial_max_translation_error_m", 0.5);
    config.initial_max_yaw_error_deg =
        reader.read_double("initial.initial_max_yaw_error_deg", 30.0);
    config.field_bounds = field_bounds;
    return config;
}

auto load_local_runtime_config(ParamReader& reader, const std::string& pointcloud_topic)
    -> LocalRuntimeConfig {
    auto config = LocalRuntimeConfig{};
    config.pointcloud_topic = pointcloud_topic;
    config.collect_duration_sec = reader.read_double("local.collect_duration_sec", 0.8);
    config.min_accumulated_points = reader.read_int("local.min_accumulated_points", 1500);
    return config;
}

auto load_local_safety_config(ParamReader& reader) -> LocalSafetyConfig {
    auto config = LocalSafetyConfig{};
    config.max_tf_correction_m = reader.read_double("local.max_tf_correction_m", 0.5);
    config.max_tf_correction_yaw_deg = reader.read_double("local.max_tf_correction_yaw_deg", 10.0);
    config.min_accept_interval_sec = reader.read_double("local.min_accept_interval_sec", 0.5);
    config.tf_lookup_timeout_sec = reader.read_double("local.tf_lookup_timeout_sec", 0.05);
    return config;
}

auto load_local_registration_config(ParamReader& reader) -> LocalRegistrationConfig {
    auto config = LocalRegistrationConfig{};
    config.coarse_iterations = reader.read_int("local.coarse_iterations", 12);
    config.precise_iterations = reader.read_int("local.precise_iterations", 12);

    config.max_correspondence_distance_m =
        reader.read_double("local.max_correspondence_distance_m", 4.0);
    config.coarse_score_threshold = reader.read_double("local.coarse_score_threshold", 0.5);

    config.yaw_window_deg = reader.read_double("local.yaw_window_deg", 30.0);
    config.coarse_yaw_step_deg = reader.read_double("local.coarse_yaw_step_deg", 15.0);

    config.sc_top_k = static_cast<std::size_t>(reader.read_positive_int("local.sc_top_k", 2));
    return config;
}

auto load_local_validation_config(ParamReader& reader, const FieldBoundsConfig& field_bounds)
    -> LocalValidationConfig {
    auto config = LocalValidationConfig{};
    config.field_bounds = field_bounds;
    config.score_threshold = reader.read_double("local.score_threshold", 0.50);
    config.min_inlier_ratio = reader.read_double("local.min_inlier_ratio", 0.10);
    config.max_distance_from_prior_m = reader.read_double("local.max_distance_from_prior_m", 5.0);
    config.max_yaw_from_prior_deg = reader.read_double("local.max_yaw_from_prior_deg", 60.0);
    return config;
}

auto load_wide_runtime_config(ParamReader& reader, const std::string& pointcloud_topic)
    -> WideRuntimeConfig {
    auto config = WideRuntimeConfig{};
    config.pointcloud_topic = pointcloud_topic;
    config.collect_duration_sec = reader.read_double("wide.collect_duration_sec", 1.5);
    config.min_accumulated_points = reader.read_int("wide.min_accumulated_points", 2000);
    return config;
}

auto load_wide_registration_config(ParamReader& reader) -> WideRegistrationConfig {
    auto config = WideRegistrationConfig{};
    config.coarse_iterations = reader.read_int("wide.coarse_iterations", 15);
    config.precise_iterations = reader.read_int("wide.precise_iterations", 25);

    config.max_correspondence_distance_m =
        reader.read_double("wide.max_correspondence_distance_m", 4.0);
    config.coarse_score_threshold = reader.read_double("wide.coarse_score_threshold", 0.35);

    config.yaw_window_deg = reader.read_double("wide.yaw_window_deg", 60.0);
    config.coarse_yaw_step_deg = reader.read_double("wide.coarse_yaw_step_deg", 18.0);

    config.sc_top_k = static_cast<std::size_t>(reader.read_positive_int("wide.sc_top_k", 5));

    config.max_candidate_count =
        static_cast<std::size_t>(reader.read_positive_int("wide.max_candidate_count", 1));
    config.rank_weight_inlier = reader.read_double("wide.rank_weight_inlier", 0.5);
    return config;
}

auto load_wide_validation_config(ParamReader& reader, const FieldBoundsConfig& field_bounds)
    -> WideValidationConfig {
    auto config = WideValidationConfig{};
    config.field_bounds = field_bounds;
    config.score_threshold = reader.read_double("wide.score_threshold", 0.35);
    config.min_inlier_ratio = reader.read_double("wide.min_inlier_ratio", 0.08);
    return config;
}

auto load_scan_context_config(ParamReader& reader) -> ScanContextConfig {
    auto config = ScanContextConfig{};
    config.num_rings = reader.read_positive_int("scan_context.num_rings", 20);
    config.num_sectors = reader.read_positive_int("scan_context.num_sectors", 60);
    config.max_radius_m = reader.read_double("scan_context.max_radius_m", 20.0);
    config.z_min_m = reader.read_double("scan_context.z_min_m", 0.15);
    config.z_max_m = reader.read_double("scan_context.z_max_m", 2.0);
    return config;
}

} // namespace

auto load_runtime_params(rclcpp::Node& node) -> RuntimeParamsBundle {
    auto reader = ParamReader{node};
    auto params = RuntimeParamsBundle{};

    params.map_path = reader.read_string("map_path", "");
    params.descriptor_path = reader.read_string("descriptor_path", "");
    params.service_name = reader.read_string("service_name", "/rmcs_relocation/relocalize");
    params.world_frame = reader.read_string("world_frame", "world");
    params.odom_frame = reader.read_string("odom_frame", "odom");
    params.base_frame = reader.read_string("base_frame", "base_link");
    params.publish_tf_rate_hz = reader.read_double("publish_tf_rate_hz", 10.0);
    const auto pointcloud_topic =
        reader.read_string("pointcloud_topic", "/cloud_registered_undistort");
    const auto field_bounds = reader.read_bounds("field_bounds", FieldBoundsConfig{});

    params.initial_runtime_config = load_initial_runtime_config(reader, pointcloud_topic);
    params.common_registration_config = load_common_registration_config(reader);
    params.initial_registration_config = load_initial_registration_config(reader);
    params.initial_validation_config =
        load_initial_validation_config(reader, params.initial_registration_config, field_bounds);

    params.local_runtime_config = load_local_runtime_config(reader, pointcloud_topic);
    params.local_safety_config = load_local_safety_config(reader);
    params.local_registration_config = load_local_registration_config(reader);
    params.local_validation_config = load_local_validation_config(reader, field_bounds);

    params.wide_runtime_config = load_wide_runtime_config(reader, pointcloud_topic);
    params.wide_registration_config = load_wide_registration_config(reader);
    params.wide_validation_config = load_wide_validation_config(reader, field_bounds);

    params.scan_context_config = load_scan_context_config(reader);

    params.log_failure_details = reader.read_bool("diagnostics.log_failure_details", false);

    return params;
}

} // namespace rmcs::location::tools

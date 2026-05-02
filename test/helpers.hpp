#pragma once

#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rmcs::location::test {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;

auto create_cube_cloud(float side_m, float density_m) -> std::shared_ptr<PointCloud>;

auto create_random_cloud(int n, float range_m) -> std::shared_ptr<PointCloud>;

auto apply_transform(
    const PointCloud& cloud,
    float dx,
    float dy,
    float dz,
    float yaw_deg) -> std::shared_ptr<PointCloud>;

} // namespace rmcs::location::test

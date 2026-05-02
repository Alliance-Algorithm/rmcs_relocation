#include "helpers.hpp"

#include <algorithm>
#include <cstdint>
#include <numbers>
#include <random>

#include <Eigen/Geometry>
#include <pcl/common/transforms.h>

namespace rmcs::location::test {

auto create_cube_cloud(float side_m, float density_m) -> std::shared_ptr<PointCloud> {
    auto cloud = std::make_shared<PointCloud>();
    const auto half = side_m / 2.0F;

    for (auto x = -half; x <= half; x += density_m) {
        for (auto y = -half; y <= half; y += density_m) {
            cloud->push_back({x, y, -half});
            cloud->push_back({x, y, half});
        }
    }

    for (auto z = -half; z <= half; z += density_m) {
        for (auto v = -half; v <= half; v += density_m) {
            cloud->push_back({v, -half, z});
            cloud->push_back({v, half, z});
            cloud->push_back({-half, v, z});
            cloud->push_back({half, v, z});
        }
    }

    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

auto create_random_cloud(int n, float range_m) -> std::shared_ptr<PointCloud> {
    auto cloud = std::make_shared<PointCloud>();
    cloud->reserve(static_cast<std::size_t>(std::max(0, n)));

    auto rng = std::mt19937{42};
    auto dist = std::uniform_real_distribution<float>{-range_m, range_m};
    for (auto i = 0; i < n; ++i)
        cloud->push_back({dist(rng), dist(rng), dist(rng)});

    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

auto apply_transform(
    const PointCloud& cloud, float dx, float dy, float dz, float yaw_deg)
    -> std::shared_ptr<PointCloud> {
    const auto rad = yaw_deg * std::numbers::pi_v<float> / 180.0F;

    auto transform = Eigen::Isometry3f::Identity();
    transform.translation() << dx, dy, dz;
    transform.rotate(Eigen::AngleAxisf{rad, Eigen::Vector3f::UnitZ()});

    auto output = std::make_shared<PointCloud>();
    pcl::transformPointCloud(cloud, *output, transform.matrix());
    output->width = static_cast<std::uint32_t>(output->size());
    output->height = 1;
    output->is_dense = cloud.is_dense;
    return output;
}

} // namespace rmcs::location::test

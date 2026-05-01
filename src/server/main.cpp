#include <rclcpp/executors/multi_threaded_executor.hpp>
#include "runtime.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rmcs::location::RelocalizationServer>();
    auto executor = rclcpp::executors::MultiThreadedExecutor(
        rclcpp::ExecutorOptions {}, 2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

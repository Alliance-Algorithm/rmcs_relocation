cmake_minimum_required(VERSION 3.16)
project(rmcs-relocation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(rmcs-relocation_compile_options
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree)
find_package(OpenMP)

find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rmcs_msgs REQUIRED)
find_package(rmcs_executor REQUIRED)

set(RMCS_LOCATION_SMALL_GICP_INCLUDE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/small_gicp/include")
if(NOT EXISTS "${RMCS_LOCATION_SMALL_GICP_INCLUDE_DIR}/small_gicp/pcl/pcl_registration.hpp")
  message(FATAL_ERROR
    "Vendored small_gicp headers are missing: ${RMCS_LOCATION_SMALL_GICP_INCLUDE_DIR}")
endif()

add_library(rmcs-relocation_small_gicp INTERFACE)
target_include_directories(rmcs-relocation_small_gicp SYSTEM INTERFACE
  "${RMCS_LOCATION_SMALL_GICP_INCLUDE_DIR}"
)
target_link_libraries(rmcs-relocation_small_gicp INTERFACE Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(rmcs-relocation_small_gicp INTERFACE OpenMP::OpenMP_CXX)
endif()

add_subdirectory(src/tools)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_library(rmcs-relocation_component SHARED
  src/component/supervisor.cpp
  $<TARGET_OBJECTS:rmcs-relocation_tools_common>
)
target_include_directories(rmcs-relocation_component PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_compile_features(rmcs-relocation_component PUBLIC cxx_std_23)
if(rmcs-relocation_compile_options)
  target_compile_options(rmcs-relocation_component PRIVATE ${rmcs-relocation_compile_options})
endif()
ament_target_dependencies(
  rmcs-relocation_component
  geometry_msgs
  pcl_conversions
  pluginlib
  rclcpp
  rmcs_executor
  rmcs_msgs
  tf2
  tf2_ros
)

add_executable(rmcs-relocation_server
  src/server/runtime.cpp
  src/server/health_monitor.cpp
  src/server/collector.cpp
  src/server/validator.cpp
  src/server/main.cpp
  $<TARGET_OBJECTS:rmcs-relocation_tools_common>
)
target_compile_features(rmcs-relocation_server PUBLIC cxx_std_23)
if(rmcs-relocation_compile_options)
  target_compile_options(rmcs-relocation_server PRIVATE ${rmcs-relocation_compile_options})
endif()
target_include_directories(rmcs-relocation_server PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  rmcs-relocation_server
  Eigen3::Eigen
  rmcs-relocation_small_gicp
  ${PCL_LIBRARIES}
)
ament_target_dependencies(
  rmcs-relocation_server
  geometry_msgs
  pcl_conversions
  rclcpp
  rmcs_msgs
  sensor_msgs
  tf2
  tf2_ros
)

pluginlib_export_plugin_description_file(rmcs_executor plugins.xml)

install(TARGETS
  rmcs-relocation_component
  rmcs-relocation_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/pcd_publisher.py
  scripts/tf_to_pose.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

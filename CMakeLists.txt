cmake_minimum_required(VERSION 3.16)
project(rmcs_relocation)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(rmcs_relocation_compile_options
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration kdtree)
find_package(OpenMP)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "srv/Relocalize.srv"
  DEPENDENCIES geometry_msgs
)

set(RMCS_LOCATION_SMALL_GICP_INCLUDE_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/small_gicp/include")
if(NOT EXISTS "${RMCS_LOCATION_SMALL_GICP_INCLUDE_DIR}/small_gicp/pcl/pcl_registration.hpp")
  message(FATAL_ERROR
    "Vendored small_gicp headers are missing: ${RMCS_LOCATION_SMALL_GICP_INCLUDE_DIR}")
endif()

add_library(rmcs_relocation_small_gicp INTERFACE)
target_include_directories(rmcs_relocation_small_gicp SYSTEM INTERFACE
  "${RMCS_LOCATION_SMALL_GICP_INCLUDE_DIR}"
)
target_link_libraries(rmcs_relocation_small_gicp INTERFACE Eigen3::Eigen)
if(OpenMP_CXX_FOUND)
  target_link_libraries(rmcs_relocation_small_gicp INTERFACE OpenMP::OpenMP_CXX)
endif()

add_subdirectory(src/tools)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

add_executable(rmcs_relocation_server
  src/server/runtime.cpp
  src/server/collector.cpp
  src/server/validator.cpp
  src/server/map_descriptor_db.cpp
  src/server/main.cpp
  $<TARGET_OBJECTS:rmcs_relocation_tools_common>
)
target_compile_features(rmcs_relocation_server PUBLIC cxx_std_23)
if(rmcs_relocation_compile_options)
  target_compile_options(rmcs_relocation_server PRIVATE ${rmcs_relocation_compile_options})
endif()
target_include_directories(rmcs_relocation_server PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(
  rmcs_relocation_server
  Eigen3::Eigen
  rmcs_relocation_small_gicp
  ${PCL_LIBRARIES}
)
ament_target_dependencies(
  rmcs_relocation_server
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
)
rosidl_target_interfaces(rmcs_relocation_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  rmcs_relocation_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/pcd_publisher.py
  scripts/tf_to_pose.py
  scripts/generate_map_descriptors.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  docs
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

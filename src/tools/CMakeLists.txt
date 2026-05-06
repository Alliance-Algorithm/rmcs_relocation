add_library(rmcs_relocation_tools_common OBJECT
  numeric_tools.cpp
  geometry_tools.cpp
  param_tools.cpp
  registration_tools.cpp
  scan_context.cpp
)
target_compile_features(rmcs_relocation_tools_common PUBLIC cxx_std_23)
if(rmcs_relocation_compile_options)
  target_compile_options(rmcs_relocation_tools_common PRIVATE ${rmcs_relocation_compile_options})
endif()
target_include_directories(rmcs_relocation_tools_common PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/.."
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(rmcs_relocation_tools_common
  rmcs_relocation_small_gicp
  ${PCL_LIBRARIES}
)
ament_target_dependencies(
  rmcs_relocation_tools_common
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
)

add_library(rmcs-relocation_tools_common OBJECT
  numeric_tools.cpp
  geometry_tools.cpp
  param_tools.cpp
  registration_tools.cpp
  scan_context.cpp
)
target_compile_features(rmcs-relocation_tools_common PUBLIC cxx_std_23)
if(rmcs-relocation_compile_options)
  target_compile_options(rmcs-relocation_tools_common PRIVATE ${rmcs-relocation_compile_options})
endif()
target_include_directories(rmcs-relocation_tools_common PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/.."
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(rmcs-relocation_tools_common
  rmcs-relocation_small_gicp
  ${PCL_LIBRARIES}
)
ament_target_dependencies(
  rmcs-relocation_tools_common
  geometry_msgs
  pcl_conversions
  rclcpp
  rmcs_msgs
  sensor_msgs
)

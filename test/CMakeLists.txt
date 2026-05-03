find_package(ament_cmake_gtest REQUIRED)

set(RMCS_RELOCATION_TEST_INCLUDE_DIRS
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/../src"
  ${PCL_INCLUDE_DIRS}
)

function(rmcs_relocation_configure_test target_name)
  target_include_directories(${target_name} PRIVATE ${RMCS_RELOCATION_TEST_INCLUDE_DIRS})
  target_compile_features(${target_name} PUBLIC cxx_std_23)
  if(rmcs-relocation_compile_options)
    target_compile_options(${target_name} PRIVATE ${rmcs-relocation_compile_options})
  endif()
endfunction()

# ---- test_validator ----
ament_add_gtest(test_validator
  test_validator.cpp
  ../src/server/validator.cpp
)
if(TARGET test_validator)
  rmcs_relocation_configure_test(test_validator)
  target_link_libraries(test_validator
    Eigen3::Eigen
  )
  ament_target_dependencies(test_validator
    sensor_msgs
  )
endif()

# ---- test_registration ----
ament_add_gtest(test_registration
  test_registration.cpp
  helpers.cpp
  ../src/tools/registration_tools.cpp
  ../src/tools/numeric_tools.cpp
)
if(TARGET test_registration)
  rmcs_relocation_configure_test(test_registration)
  target_link_libraries(test_registration
    Eigen3::Eigen
    rmcs-relocation_small_gicp
    ${PCL_LIBRARIES}
  )
  ament_target_dependencies(test_registration
    pcl_conversions
    rclcpp
    sensor_msgs
  )
endif()

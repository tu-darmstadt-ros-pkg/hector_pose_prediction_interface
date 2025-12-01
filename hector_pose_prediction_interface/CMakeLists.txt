cmake_minimum_required(VERSION 3.8)
project(
  hector_pose_prediction_interface
  VERSION 0.8
  LANGUAGES CXX)

# Use a modern C++ standard in ROS 2
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hector_math REQUIRED)
find_package(hector_stability_metrics REQUIRED)
find_package(Eigen3 REQUIRED)

set(THIS_PACKAGE_DEPENDENCIES hector_math hector_stability_metrics Eigen3)

# ##############################################################################
# Build ##
# ##############################################################################

# Header-only interface library
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(
  ${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

ament_target_dependencies(${PROJECT_NAME} INTERFACE hector_math
                          hector_stability_metrics)

# ##############################################################################
# Install ##
# ##############################################################################

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})

# ##############################################################################
# Testing ##
# ##############################################################################

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # If you later port the tests, use something like:
  #
  # ament_add_gtest(test_pose test/test_pose.cpp ) if(TARGET test_pose)
  # target_link_libraries(test_pose ${PROJECT_NAME})
  # ament_target_dependencies(test_pose ${THIS_PACKAGE_DEPENDENCIES}) endif()
  #
  # (rostest/launch-based tests would be replaced with launch_testing in ROS 2.)

  if(ENABLE_COVERAGE_TESTING)
    find_package(code_coverage REQUIRED)
    include(CodeCoverage)
    append_coverage_compiler_flags()
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*" "*/build/*")
    add_code_coverage(NAME ${PROJECT_NAME}_coverage EXECUTABLE ctest)
  endif()
endif()

# ##############################################################################
# Export##
# ##############################################################################

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()

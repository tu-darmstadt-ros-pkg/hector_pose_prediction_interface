cmake_minimum_required(VERSION 3.8)
project(hector_pose_prediction_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hector_pose_prediction_interface REQUIRED)
find_package(hector_math REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(hector_math_ros REQUIRED)

set(THIS_PACKAGE_DEPENDENCIES hector_pose_prediction_interface hector_math
                              visualization_msgs hector_math_ros)

# ##############################################################################
# Build ##
# ##############################################################################

# Header-only interface library
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(
  ${PROJECT_NAME} INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} INTERFACE
                          ${THIS_PACKAGE_DEPENDENCIES})

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
# Export##
# ##############################################################################

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_DEPENDENCIES})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()

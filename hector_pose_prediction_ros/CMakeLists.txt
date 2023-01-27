cmake_minimum_required(VERSION 3.0.2)
project(hector_pose_prediction_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hector_pose_prediction_interface
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES hector_pose_prediction_ros
 CATKIN_DEPENDS hector_pose_prediction_interface
)

###########
## Build ##
###########

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} INTERFACE ${EIGEN3_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

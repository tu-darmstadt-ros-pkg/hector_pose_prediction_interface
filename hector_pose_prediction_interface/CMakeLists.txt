cmake_minimum_required(VERSION 3.0.2)
project(hector_pose_prediction_interface)

find_package(catkin REQUIRED COMPONENTS hector_stability_metrics)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS hector_stability_metrics
  DEPENDS EIGEN3
)

###########
## Build ##
###########

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

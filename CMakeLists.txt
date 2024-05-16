cmake_minimum_required(VERSION 3.0.2)
project(aprilslamcpp)

# Set GTSAM_DIR to the directory containing GTSAMConfig.cmake
set(GTSAM_DIR "/home/shu/GTSAM/build")
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  tf2_ros
)

find_package(Eigen3 REQUIRED) # Find Eigen

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs tf2_ros nav_msgs 
  DEPENDS GTSAM
)


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${PROJECT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME} src/aprilslamcpp.cpp src/publishing_utils.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  gtsam 
  tbb
)

#############
## Install ##
#############

install(
  TARGETS aprilslamcpp 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_AprilSlamCPP.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

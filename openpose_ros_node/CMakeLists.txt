cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros_node)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenPose REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "OpenPose Include dir at ${OpenPose_INCLUDE_DIRS}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")
message(STATUS "CMAKE Module Path at ${CMAKE_MODULE_PATH}")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
add_definitions(-DUSE_CAFFE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cv_bridge
  sensor_msgs
  image_transport
  openpose_ros_msgs
  openpose_ros_srvs
)

find_package(CUDA REQUIRED)
find_package(gflags REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    openpose_ros_srvs
)

#set(OPENPOSE_DIR "$ENV{OPENPOSE_HOME}")
#
#if(OPENPOSE_DIR)               # <--- Use 'BOOST_DIR', not 'DEFINED ${BOOST_DIR}'
#    message(STATUS "OPENPOSE_HOME at ${OPENPOSE_DIR}")
#else()
#    message(FATAL_ERROR "Undefined OPENPOSE_HOME env var.")
#endif()

# include_directories(include)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CUDA_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS}
  
)

add_executable(${PROJECT_NAME} src/openpose_ros_node.cpp )

# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenPose_LIBS}
  ${OpenCV_LIBS}
  gflags
)


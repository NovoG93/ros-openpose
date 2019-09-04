cmake_minimum_required(VERSION 2.8.3)
project(openpose_ros_node)

set(CMAKE_BUILD_TYPE Debug)

find_package(OpenPose REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
message(STATUS "OpenPose Include dir at ${OpenPose_INCLUDE_DIRS}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/Modules/")
message(STATUS "CMAKE Module Path at ${CMAKE_MODULE_PATH}")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++17)
add_definitions(-DUSE_CAFFE)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
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

## Generate the dynamic_reconfigure
generate_dynamic_reconfigure_options(
   cfg/config.cfg
   #...
)


catkin_package(
  CATKIN_DEPENDS
    std_msgs
    openpose_ros_srvs
)



include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${CUDA_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS}
  
)


add_executable(openpose_node src/main.cpp src/openpose_ros.cpp)
target_link_libraries(openpose_node ${catkin_LIBRARIES} ${OpenPose_LIBS} ${OpenCV_LIBS} yaml-cpp)
add_dependencies(openpose_node yaml-cpp)


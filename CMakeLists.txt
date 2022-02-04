cmake_minimum_required(VERSION 2.8.3)
project(dyros_pcv_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    roslib
    std_msgs
    sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES dyros_pcv_controller
#  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs
#  DEPENDS system_lib
)

find_package(PkgConfig QUIET)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

add_definitions (-DNON_MATLAB_PARSING)
add_definitions (-DMAX_EXT_API_CONNECTIONS=255)
#add_definitions (-DDO_NOT_USE_SHARED_MEMORY)

set (SOURCES
	src/main.cpp
  src/caster.cpp
  src/vehicle.cpp
  src/mobile_controller.cpp
  src/ros_node.cpp
  
  include/math_type_define.h
  include/linux_terminal_tool
  include/pcv_mtx_utils.h
)

add_executable(dyros_pcv_controller ${SOURCES})

target_link_libraries(dyros_pcv_controller
	pthread
	rt
  ${YAML_CPP_LIBRARIES}
  ${catkin_LIBRARIES}
)

set(CMAKE_BUILD_TYPE RELEASE)

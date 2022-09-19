cmake_minimum_required(VERSION 2.8.3)
project(grid_map)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")



find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  tf
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES grid_map
 CATKIN_DEPENDS roscpp rospy std_msgs tf 
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(laser_detect_node src/laser_detect.cpp)
#add_executable(local_map_manager_node src/local_map_manager.cpp)

link_directories(${PCL_LIBRARY_DIRS})

add_library( grid_map 
    #src/laser_detect.cpp 
    src/global_map_manager.cpp
    )

target_link_libraries(laser_detect_node
    ${catkin_LIBRARIES}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_LIBRARIES}
 )

target_link_libraries( grid_map
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    )  

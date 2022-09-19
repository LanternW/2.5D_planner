cmake_minimum_required(VERSION 3.0.2)
project(detector)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS

  grid_map
  cv_bridge
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
  ugv_planner
)

catkin_package( 
INCLUDE_DIRS include
DEPENDS OpenCV Eigen
CATKIN_DEPENDS 

)

include_directories(
  # include
  SYSTEM 
  include 
  ${catkin_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/include
  ${Eigen_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

add_executable(laser_pointer_node
  src/laser_pointer.cpp 
  )

add_dependencies(laser_pointer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(laser_pointer_node 
  ${catkin_LIBRARIES}
  )

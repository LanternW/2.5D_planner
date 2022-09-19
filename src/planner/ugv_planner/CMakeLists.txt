cmake_minimum_required(VERSION 2.8.3)
project(ugv_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")



find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
#find_package(ompl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  visualization_msgs
  cv_bridge
  #plan_env
  #path_searching
  bspline_opt
  grid_map
  traj_utils
  message_generation
)


add_message_files(FILES
	DVHcontrol.msg
  PolyhedronMSG.msg
  PolyhedronArrayMSG.msg
  Polynome.msg
)
generate_messages(DEPENDENCIES
	std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 DEPENDS OpenCV Eigen Boost
 #CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils
 CATKIN_DEPENDS bspline_opt traj_utils grid_map
 #CATKIN_DEPENDS traj_utils grid_map
 message_runtime
)
include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} 
    ${PROJECT_SOURCE_DIR}/include
    ${Eigen_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)


# add_library( car_search 
#     src/car_search.cpp
#     )
# target_link_libraries( car_search
#     ${catkin_LIBRARIES}
#     ompl
# )
# add_executable(car_node
#     src/car_search_node.cpp
# )
# target_link_libraries(car_node
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
#   car_search
# )


add_executable(ugv_planner_node
  src/ugv_planner_node.cpp 
  src/planner_manager.cpp
  src/traj_visualization.cpp

  src/bezier_base.cpp
  src/lan_bezier_optimizer.cpp
  src/minco_optimizer.cpp
  )

add_dependencies(ugv_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(ugv_planner_node 
  ${catkin_LIBRARIES}
  ooqpgensparse 
    ooqpsparse 
    ooqpgondzio 
    ooqpbase blas ma27 gfortran f2c 

  )


add_executable(ugv_vis_node
  src/ugv_manager.cpp
)

target_link_libraries(ugv_vis_node
${catkin_LIBRARIES}
)


add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})

# add_library()

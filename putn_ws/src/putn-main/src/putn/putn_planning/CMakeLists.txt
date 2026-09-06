cmake_minimum_required(VERSION 3.0.2)
project(putn)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rviz_visual_tools
  std_msgs
  rover_msgs
)

find_package(PCL REQUIRED)

catkin_package(
INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall") # -Wextra -Werror

add_executable(global_planning_node
    src/global_planning_node.cpp
    src/PUTN_planner.cpp
    src/PUTN_classes.cpp
    src/PUTN_vis.cpp)

target_link_libraries(global_planning_node 
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES} 
)




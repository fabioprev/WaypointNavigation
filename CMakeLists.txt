cmake_minimum_required(VERSION 2.8.3)

project(WaypointNavigation)

set(ROS_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE WaypointNavigation_src "src/*.cpp")

add_executable(WaypointNavigation ${WaypointNavigation_src})
target_link_libraries(WaypointNavigation ${catkin_LIBRARIES})

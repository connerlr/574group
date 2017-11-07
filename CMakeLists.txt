cmake_minimum_required(VERSION 2.8.3)

project(project_03)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs nav_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(project_03 project_03.cpp)

target_link_libraries(project_03 ${catkin_LIBRARIES})

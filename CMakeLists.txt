cmake_minimum_required(VERSION 2.8.3)

project(csce574hw2)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs nav_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pdc pdc.cpp)

target_link_libraries(pdc ${catkin_LIBRARIES})

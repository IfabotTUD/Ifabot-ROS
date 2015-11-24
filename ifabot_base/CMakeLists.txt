cmake_minimum_required(VERSION 2.8.3)
project(ifabot_base)

find_package(catkin REQUIRED COMPONENTS controller_manager roscpp sensor_msgs std_msgs geometry_msgs)
find_package(Boost REQUIRED COMPONENTS system thread chrono)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(ifabot_base src/ifabot_base.cpp src/ifabot_hardware.cpp)
target_link_libraries(ifabot_base ${catkin_LIBRARIES} boost_system boost_thread boost_chrono)

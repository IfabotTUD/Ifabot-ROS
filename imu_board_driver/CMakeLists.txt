cmake_minimum_required(VERSION 2.8.3)
project(imu_board_driver)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp message_generation sensor_msgs std_msgs)
find_package(Boost REQUIRED COMPONENTS system thread)

add_service_files(FILES Calibrate.srv)

generate_messages()

generate_dynamic_reconfigure_options(
  cfg/dynamic_reconfigurable_parameter.cfg
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS} src/)

add_executable(imu_board_driver src/IMUBoardDriver.cpp)
add_dependencies(imu_board_driver ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)
target_link_libraries(imu_board_driver ${catkin_LIBRARIES} boost_system boost_thread ifabot)


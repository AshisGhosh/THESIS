cmake_minimum_required(VERSION 2.8.3)
project(rrbot_arm)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()

add_executable(armswing src/armswing.cpp)
target_link_libraries(armswing ${catkin_LIBRARIES})

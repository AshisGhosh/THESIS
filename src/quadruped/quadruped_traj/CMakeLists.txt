cmake_minimum_required(VERSION 2.8.3)
project(quadruped_traj)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


add_executable(movement src/movement.cpp)

target_link_libraries(movement ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(movement_adj src/movement_adj.cpp)

target_link_libraries(movement_adj ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(move src/move.cpp)

target_link_libraries(move ${catkin_LIBRARIES})

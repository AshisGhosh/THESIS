cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(robot_cog SHARED robot_cog.cc)
target_link_libraries(robot_cog ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})


add_library(surfaces SHARED surfaces.cc)
target_link_libraries(surfaces ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(surfaceremove SHARED surfaceremove.cc)
target_link_libraries(surfaceremove ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

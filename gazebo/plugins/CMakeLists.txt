cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(wind_animation)

# Find packages

find_package(gazebo REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Create libraries and executables

add_library(wind_animation SHARED wind_animation.cc)
target_link_libraries(wind_animation ${GAZEBO_LIBRARIES})

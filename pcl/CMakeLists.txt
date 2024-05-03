cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(point_cloud_localization)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (localization localization.cpp)
target_link_libraries (localization ${PCL_LIBRARIES})

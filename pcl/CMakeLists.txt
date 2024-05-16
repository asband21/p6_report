cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(point_cloud_localization)

option(BUILD_PYTHON_BINDING "Build the Python binding." OFF)
option(USE_BOOST "Use Boost for shared pointers" OFF)  

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


if(USE_BOOST)
    find_package(Boost REQUIRED)
    add_definitions(-DUSE_BOOST)
endif()

if(BUILD_PYTHON_BINDING)
    # Build the localization function as a shared library
    add_library(localization SHARED localization.cpp)
    target_link_libraries(localization ${PCL_LIBRARIES})
    # Ensure we don't add a 'lib' prefix on Windows
    set_target_properties(localization PROPERTIES PREFIX "")
else()
    add_executable(localization localization.cpp)
    target_link_libraries(localization ${PCL_LIBRARIES})
endif()


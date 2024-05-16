cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(point_cloud_localization)

option(BUILD_PYTHON_BINDING "Build the Python binding." OFF)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Check for boost
find_package(Boost QUIET)
if (Boost_FOUND)
    set(USE_BOOST ON)
    add_definitions(-DUSE_BOOST)
else()
    # Check for pcl/memory.h
    include(CheckIncludeFile)
    check_include_file("pcl/memory.h" HAVE_PCL_MEMORY)
    if (HAVE_PCL_MEMORY)
        add_definitions(-DHAVE_PCL_MEMORY)
    else()
        message(FATAL_ERROR "Neither boost nor pcl/memory.h is available")
    endif()
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


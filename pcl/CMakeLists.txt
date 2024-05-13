cmake_minimum_required(VERSION 3.10)
project(ICPExample)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ICPExample main.cpp ICP.cpp ICPUtils.cpp)
target_link_libraries(ICPExample ${PCL_LIBRARIES})

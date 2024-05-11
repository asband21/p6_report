cmake_minimum_required(VERSION 3.10)
project(test)

# Require C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add source files
add_library(test MODULE test.cpp)

# Include directories
target_include_directories(test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# If needed, link against math library
find_package(Threads REQUIRED)
target_link_libraries(test ${CMAKE_THREAD_LIBS_INIT} m)


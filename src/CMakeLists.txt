cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(bollcnt CXX)
file(GLOB SOURCES
    headers/*.h
    modules/*.cpp
)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(PCL 1.6 REQUIRED COMPONENTS common io features filters octree search kdtree sample_consensus segmentation)
include_directories(${PCL_INCLUDE_DIRS} .)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(bollcnt main.cpp ${SOURCES})
target_link_libraries(bollcnt ${PCL_LIBRARIES})
set_property(TARGET bollcnt PROPERTY CXX_STANDARD 11)
set_property(TARGET bollcnt PROPERTY CXX_STANDARD_REQUIRED ON)

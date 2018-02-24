cmake_minimum_required (VERSION 2.8.11)

project (NN-ROOMBA)

set(CMAKE_CXX_COMPILER "/usr/bin/g++-5")

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
endif()

#find_package(gflags REQUIRED)

message(STATUS "Boost library version ${Boost_VERSION}")

add_subdirectory (src)
add_subdirectory (libs)



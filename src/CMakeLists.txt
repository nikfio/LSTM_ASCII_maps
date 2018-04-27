cmake_minimum_required (VERSION 3.2 FATAL_ERROR)

project(nn_ascii_maps VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_COMPILER "/usr/bin/g++-5")

include(GNUInstallDirs)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++0x")
endif()

include_directories(${PROJECT_BIN_DIR}/include)

add_subdirectory(src)
add_subdirectory(app)




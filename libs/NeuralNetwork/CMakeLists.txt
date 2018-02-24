# adding subdirectories for a better organization

# cmake part to link caffe and all related libraries/dependencies
if(ON)
  if(NOT OpenCV_FOUND)
    set(Caffe_OpenCV_CONFIG_PATH "/usr/share/OpenCV")
    if(Caffe_OpenCV_CONFIG_PATH)
      get_filename_component(Caffe_OpenCV_CONFIG_PATH ${Caffe_OpenCV_CONFIG_PATH} ABSOLUTE)

      if(EXISTS ${Caffe_OpenCV_CONFIG_PATH} AND NOT TARGET opencv_core)
        message(STATUS "Caffe: using OpenCV config from ${Caffe_OpenCV_CONFIG_PATH}")
        include(${Caffe_OpenCV_CONFIG_PATH}/OpenCVModules.cmake) 
      endif()

    else()
      find_package(OpenCV REQUIRED)
    endif()
    unset(Caffe_OpenCV_CONFIG_PATH)
  endif()
endif()

# Compute paths
get_filename_component(Caffe_CMAKE_DIR "/usr/local/caffe/cmake" PATH)
#FIND_PACKAGE(Caffe)
set(Caffe_INCLUDE_DIRS /usr/local/caffe/include)
list(APPEND Caffe_INCLUDE_DIRS "/usr/include")
list(APPEND Caffe_INCLUDE_DIRS "/usr/local/cuda/include")
list(APPEND Caffe_INCLUDE_DIRS "/usr/include/opencv")
list(APPEND Caffe_INCLUDE_DIRS "/usr/include/atlas")
list(APPEND Caffe_INCLUDE_DIRS "/usr/local/caffe/build/src/")

include_directories(${Caffe_INCLUDE_DIRS})

# Definitions
set(Caffe_DEFINITIONS "-DUSE_OPENCV;-DUSE_LMDB;-DUSE_LEVELDB")

set(CAFFE_DIR /usr/local/caffe)

set(LIBRARY  -L${CAFFE_DIR}/build/lib -L/usr/local/Cellar/opencv/2.4.11_1/lib/ -lglog -lprotobuf -lpython2.7 -lcaffe -lm -lpthread -lopencv_core -lopencv_imgproc -lopencv_highgui -lboost_system)


# adding all libraries needed
add_library (Common_net Common_net.cpp)

add_library (Build_set Build_set.cpp)

add_library (TrainValidateRNN TrainValidateRNN.cpp OnlineRNN.cpp TrainVal_db.cpp build_db.cpp)

# linking libraries needed for source files
target_link_libraries(Common_net ${LIBRARY} Scenario Supervisor)

target_link_libraries(Build_set Scenario Supervisor)

target_link_libraries(TrainValidateRNN ${LIBRARY} Scenario Supervisor)


# target_include_directories (Map PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories (Common_net PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories (Build_set PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories (TrainValidateRNN PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} glog)








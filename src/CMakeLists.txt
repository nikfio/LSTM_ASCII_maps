
# creating executable for server and clients
add_executable (TestAstar TestAstar.cpp)

add_executable (TestNeuralNetwork TestNeuralNetwork.cpp)

add_executable (TestRRT TestRRT.cpp)


# linking all libraries needed

target_link_libraries (TestAstar Scenario Supervisor ${Boost_LIBRARIES})

target_link_libraries (TestRRT Scenario Supervisor ${Boost_LIBRARIES} RRT)

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

target_link_libraries (TestNeuralNetwork gflags -lboost_filesystem ${LIBRARY} Scenario Supervisor Build_set TrainValidateRNN ) 				




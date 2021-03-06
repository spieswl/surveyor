cmake_minimum_required(VERSION 2.8.3)
project(surveyor)


##### Flags #####
set(EXECUTABLE_OUTPUT_PATH bin)
set(SURVEYOR_DSO_SRC src/surveyor_dso/surveyor_dso.cpp)
set(SURVEYOR_REMODE_SRC src/surveyor_remode/surveyor_remode.cpp)

set(CMAKE_CXX_FLAGS "${SSE_FLAGS} -O2 -g -std=c++0x -march=native -fno-omit-frame-pointer")


##### Catkin main components #####
find_package(catkin REQUIRED 
    COMPONENTS camera_calibration cv_bridge image_transport message_generation roscpp roslib rospy geometry_msgs pcl_ros std_msgs sensor_msgs)


##### Required packages #####
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3 -use_fast_math)
set(CUDA_PROPAGATE_HOST_FLAGS OFF)

find_library(DSO_LIBRARY
    NAMES dso
    PATHS $ENV{DSO_PATH}/build/lib)
find_library(REMODE_LIBRARY
    NAMES rpg_open_remode
    PATHS $ENV{REMODE_PATH}/lib)
find_library(REMODE_CUDA_LIBRARY
    NAMES rpg_open_remode_cuda
    PATHS $ENV{REMODE_PATH}/lib)


##### Optional packages #####
# NONE AT PRESENT


##### Supplemental includes #####
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    $ENV{DSO_PATH}/src
    $ENV{DSO_PATH}/thirdparty/Sophus
    $ENV{REMODE_PATH}/src
    $ENV{REMODE_PATH}/include
)


##### Package build info export #####
catkin_package(
    DEPENDS OpenCV
    CATKIN_DEPENDS camera_calibration cv_bridge image_transport roscpp roslib rospy geometry_msgs pcl_ros std_msgs sensor_msgs
    INCLUDE_DIRS include
    LIBRARIES ${DSO_LIBRARY} ${REMODE_LIBRARY} ${REMODE_CUDA_LIBRARY}
)


##### Build directives #####
add_executable(surveyor_dso src/surveyor_dso/surveyor_dso.cpp ${SURVEYOR_DSO_SRC})
target_link_libraries(surveyor_dso ${DSO_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Pangolin_LIBRARIES})

add_executable(surveyor_remode src/surveyor_remode/surveyor_remode.cpp ${SURVEYOR_REMODE_SRC})
target_link_libraries(surveyor_remode ${REMODE_LIBRARY} ${REMODE_CUDA_LIBRARY} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${CUDA_LIBRARIES})
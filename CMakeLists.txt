cmake_minimum_required(VERSION 2.8.3)
project(surveyor)


##### Flags #####
set(DSO_PATH $ENV{DSO_PATH})
set(EXECUTABLE_OUTPUT_PATH bin)
set(SURVEYOR_DSO_WRAPPER src/surveyor_dso/surveyor_dso.cpp)

set(CMAKE_CXX_FLAGS "${SSE_FLAGS} -O2 -g -std=c++0x -march=native -fno-omit-frame-pointer")


##### Catkin main components #####
find_package(catkin REQUIRED 
    COMPONENTS camera_calibration cv_bridge message_generation roscpp roslib rospy geometry_msgs std_msgs sensor_msgs)


##### Required packages #####
find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(${Boost_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_library(DSO_LIBRARY dso ${DSO_PATH}/build/lib)


##### Optional packages #####
# NONE AT PRESENT


##### Supplemental includes #####
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${DSO_PATH}/src
    ${DSO_PATH}/thirdparty/Sophus
)


##### Package build info export #####
catkin_package()


##### Build directives #####
add_executable(surveyor_dso src/surveyor_dso/surveyor_dso.cpp ${SURVEYOR_DSO_WRAPPER})
target_link_libraries(surveyor_dso ${DSO_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
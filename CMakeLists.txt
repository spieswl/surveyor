cmake_minimum_required(VERSION 2.8.3)
project(surveyor)

####################

set(ROS_BUILD_TYPE Release)
set(EXECUTABLE_OUTPUT_PATH bin)
set(DSO_PATH $ENV{DSO_PATH})
message("\n\n---- LOOKING FOR DSO at\n\"${DSO_PATH}\"")

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    roslib
    rospy
    std_msgs
    sensor_msgs
    cv_bridge
    camera_calibration
)

# NOTE: Will eventually be removed when Pangolin is no longer needed
find_package(Pangolin 0.2 REQUIRED)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

find_package(OpenCV REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

find_package(Eigen3 REQUIRED)

find_package(Boost COMPONENTS system thread) 

find_library(DSO_LIBRARY dso ${DSO_PATH}/build/lib)

####################

set(SOURCE_FILES src/surveyor_dso/surveyor_dso.cpp)

add_executable(surveyor_dso src/surveyor_dso/surveyor_dso.cpp ${SOURCE_FILES})

generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
    geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${DSO_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES}
    CATKIN_DEPENDS message_generation roscpp roslib rospy std_msgs sensor_msgs cv_bridge camera_calibration
    DEPENDS Eigen3 OpenCV Pangolin
)

include_directories(
    ${PROJECT_SOURCE_DIR}/src
    ${DSO_PATH}/src
    ${DSO_PATH}/thirdparty/Sophus
    ${OpenCV_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

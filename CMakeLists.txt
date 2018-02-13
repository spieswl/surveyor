cmake_minimum_required(VERSION 2.8.3)
project(surveyor)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
  sensor_msgs
  image_publisher
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package()

include_directories()

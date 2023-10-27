cmake_minimum_required(VERSION 3.0.2)
project(sound_processing)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

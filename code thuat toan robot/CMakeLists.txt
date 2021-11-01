cmake_minimum_required(VERSION 2.8.3)

project(rbx2_tasks)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  
  message_generation
)
add_message_files(
  FILES
  PositionMove.msg
)
catkin_python_setup()
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime 
#  INCLUDE_DIRS include
#  LIBRARIES basics
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)



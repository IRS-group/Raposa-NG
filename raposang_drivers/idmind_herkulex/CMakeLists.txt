cmake_minimum_required(VERSION 2.8.3)
project(idmind_herkulex)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
    rospy
    std_msgs
    message_generation
    message_runtime
)
add_message_files(
  FILES
  LedCommand.msg
  MotorCommand.msg
  TorqueCommand.msg
  Motors.msg
  MotorStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES herkulex
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

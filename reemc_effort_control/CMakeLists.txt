cmake_minimum_required(VERSION 2.8.3)
project(reemc_effort_control)

find_package(catkin REQUIRED COMPONENTS
  force_control_reemc
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES reemc_effort_control
#  CATKIN_DEPENDS force_control_reemc roscpp
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


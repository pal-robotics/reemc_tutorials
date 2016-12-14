cmake_minimum_required(VERSION 2.8.3)
project(reemc_wbc_tutorials)

find_package(catkin REQUIRED COMPONENTS
  pal_wbc_controller
  wbc_tasks
  pluginlib
  pal_robot_tools
  actionlib
  control_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
  CATKIN_DEPENDS pal_wbc_controller wbc_tasks pluginlib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(reemc_wbc src/reemc_stack_boths_hands_head.cpp
                      src/reemc_weighted_stack_boths_hands_head.cpp
                      src/reemc_stack_joints.cpp
                      src/reemc_stack_ball_grasping.cpp
                      src/reemc_stack_humanoids_dance.cpp
                      src/reemc_stack_admitance.cpp)
target_link_libraries(reemc_wbc ${catkin_LIBRARIES})


add_executable(wbc_reemc_hardware_test src/wbc_reemc_hardware_test.cpp)
target_link_libraries(wbc_reemc_hardware_test ${catkin_LIBRARIES})

add_executable(wbc_grasping_hardware_test src/wbc_grasping_hardware_test.cpp)
target_link_libraries(wbc_grasping_hardware_test ${catkin_LIBRARIES})


if(CATKIN_ENABLE_TESTING)

  #catkin_add_gtest(wbc_kinematic_reemc_gtest gtest/wbc_kinematic_reemc_test.cpp gtest/reemc_stacks_gtest.cpp)
  #target_link_libraries(wbc_kinematic_reemc_gtest ${PROJECT_NAME} gtest)

endif()

#### Install ###

install(TARGETS reemc_wbc wbc_reemc_hardware_test
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES reemc_wbc_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)


install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

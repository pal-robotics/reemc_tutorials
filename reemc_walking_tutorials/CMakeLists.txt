cmake_minimum_required(VERSION 2.8.3)
project(reemc_walking_tutorials)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  walking_controller
  walking_msgs
  rostest
  gazebo_msgs
  actionlib
  humanoid_nav_msgs
  eigen_checks
  pal_robot_tools
)

catkin_package(
  INCLUDE_DIRS include test
  CATKIN_DEPENDS rostest eigen_checks walking_msgs gazebo_msgs actionlib humanoid_nav_msgs pal_robot_tools walking_controller actionlib
)

###########
## Build ##
###########

include_directories(
  include test
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}  src/LinkStatesListener.cpp)

add_executable(walk_client_example_topic src/walk_client_example_topic.cpp)
target_link_libraries(walk_client_example_topic  ${catkin_LIBRARIES})

add_executable(walk_client_example_service src/walk_client_example_service.cpp)
add_dependencies(walk_client_example_service walking_msgs_generate_messages_cpp)
target_link_libraries(walk_client_example_service  ${catkin_LIBRARIES})

add_executable(walk_client_example_action src/walk_client_example_action.cpp)
add_dependencies(walk_client_example_action humanoid_nav_msgs_generate_messages_cpp)
target_link_libraries(walk_client_example_action  ${catkin_LIBRARIES})

add_executable(walk_safety_example src/safety_example.cpp)
target_link_libraries(walk_safety_example  ${catkin_LIBRARIES})

add_executable(walk_ik_example src/ik_example.cpp)
target_link_libraries(walk_ik_example  ${catkin_LIBRARIES})


if(CATKIN_ENABLE_TESTING)

add_rostest_gtest(test_walking_service test/reemc_emptyworld_controllers.test test/five_steps_test.cpp)
target_link_libraries(test_walking_service ${catkin_LIBRARIES} ${PROJECT_NAME} gtest pthread)

endif()

#############
## Install ##
#############
install(TARGETS walk_client_example_topic
                walk_client_example_service
                walk_client_example_action
                walk_safety_example
                walk_ik_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


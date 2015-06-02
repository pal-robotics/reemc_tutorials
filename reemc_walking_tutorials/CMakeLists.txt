cmake_minimum_required(VERSION 2.8.3)
project(reemc_walking_tutorials)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  walking_controller
  walking_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES reemc_walking_tutorials
#  CATKIN_DEPENDS walking_controller walking_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(walk_client_example_topic src/walk_client_example_topic.cpp)
target_link_libraries(walk_client_example_topic  ${catkin_LIBRARIES})

add_executable(walk_client_example_service src/walk_client_example_service.cpp)
add_dependencies(walk_client_example_service walking_msgs_generate_messages_cpp)
target_link_libraries(walk_client_example_service  ${catkin_LIBRARIES})

add_executable(walk_client_example_action src/walk_client_example_action.cpp)
add_dependencies(walk_client_example_action humanoid_nav_msgs_generate_messages_cpp)
target_link_libraries(walk_client_example_action  ${catkin_LIBRARIES})


#############
## Install ##
#############
install(TARGETS ${targets}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config data launch)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

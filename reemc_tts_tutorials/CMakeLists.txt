cmake_minimum_required(VERSION 2.8.3)
project(reemc_tts_tutorials)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  pal_interaction_msgs
  roscpp
)


catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(tts_client src/tts_client.cpp)
add_dependencies(tts_client pal_interaction_msgs_generate_messages_cpp)
target_link_libraries(tts_client ${catkin_LIBRARIES})


#############
## Install ##
#############
install(TARGETS tts_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

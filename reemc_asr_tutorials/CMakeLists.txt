cmake_minimum_required(VERSION 2.8.3)
project(reemc_asr_tutorials)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  pal_interaction_msgs
  geometry_msgs
  roscpp
)

find_package(Boost REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(progs
  asr_client
  asr_walking
  asr_walking_exercise
  asr_walking_solved
)

foreach(prog ${progs})
  set(target reemc_tutorial_${prog})
  add_executable(${target} src/${prog}.cpp)
  set_target_properties(${target} PROPERTIES OUTPUT_NAME ${prog})
  target_link_libraries(${target} ${catkin_LIBRARIES})
  list(APPEND targets ${target})
endforeach()


#############
## Install ##
#############

install(
  PROGRAMS
    scripts/jackClientStart.sh
    scripts/jackServerStart.sh
    scripts/jackConnect.sh
    scripts/jackClientStop.sh
    scripts/jackServerStop.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${targets}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

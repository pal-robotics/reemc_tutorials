cmake_minimum_required(VERSION 2.8.3)
project(reemc_motion_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  play_motion_msgs
)

catkin_python_setup()
catkin_package()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

set(progs
  move_left_arm_without_planning
  move_left_arm_with_planning
)

foreach(prog ${progs})
  set(target reemc_tuto_${prog})
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
    scripts/test_upper_body_joints
    scripts/test_lower_body_joints
    scripts/test_joints_helpers.bash
    scripts/back_to_home.py
    scripts/motion_sequence
    scripts/simple_grasp
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${targets}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

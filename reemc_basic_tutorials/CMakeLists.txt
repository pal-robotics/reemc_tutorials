cmake_minimum_required(VERSION 2.8.3)
project(reemc_basic_tutorials)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  controller_manager_msgs
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  pal_camera_client
  pal_detection_msgs
  pal_interaction_msgs
  play_motion_msgs
  roscpp
  sensor_msgs
)

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

###########
## Build ##
###########

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(progs
  add_textured_object
  face_detection
  face_enrollment
  face_recognition
  fullbody_person_2d_detection
  look_to_object
  look_to_point
  move_left_arm_without_planning
  move_left_arm_with_planning
  reach_pose
  remote_monocular_image_visualization
  remote_stereo_image_visualization
  textured_object_detection
  tts_client
)

foreach(prog ${progs})
  set(target reemc_tuto_${prog})
  add_executable(${target} src/${prog}.cpp)
  set_target_properties(${target} PROPERTIES OUTPUT_NAME ${prog})
  target_link_libraries(${target} ${catkin_LIBRARIES} ${OpenCV_LIBS})
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
    scripts/look_at_detected_face.py
    scripts/back_to_home.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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

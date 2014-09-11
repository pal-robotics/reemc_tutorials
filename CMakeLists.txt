cmake_minimum_required(VERSION 2.8.3)
project(reemc_tutorials)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  controller_manager_msgs
  cv_bridge
  geometry_msgs
  humanoid_nav_msgs
  image_transport
  message_filters
  pal_camera_client
  pal_detection_msgs
  pal_interaction_msgs
  play_motion_msgs
  roscpp
  sensor_msgs
  walking_msgs
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

set(targets
  add_textured_object
  asr_client
  asr_walking
  asr_walking_exercise
  asr_walking_solved
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
  walk_client_example_action
  walk_client_example_service
  walk_client_example_topic
)

foreach(target ${targets})
  add_executable(${target} src/${target}.cpp)
  target_link_libraries(${target} ${catkin_LIBRARIES} ${OpenCV_LIBS})
endforeach()

#############
## Install ##
#############

install(
  PROGRAMS
    scripts/jackClientStart.sh
    scripts/test_upper_body_joints
    scripts/sitActionStart.sh
    scripts/jackServerStart.sh
    scripts/test_lower_body_joints
    scripts/sitActionEnd.sh
    scripts/test_joints_helpers.bash
    scripts/look_at_detected_face.py
    scripts/doorStepWalkOverStart.sh
    scripts/back_to_home.py
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

foreach(dir config data launch)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

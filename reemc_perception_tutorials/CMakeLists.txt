cmake_minimum_required(VERSION 2.8.3)
project(reemc_perception_tutorials)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  pal_camera_client
  pal_detection_msgs
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
  textured_object_detection
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
    scripts/look_at_detected_face.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${targets}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir data launch)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

cmake_minimum_required(VERSION 2.8.3)
project(reemc_vision_tutorials)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  pal_camera_client
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
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(progs
  look_to_point
  remote_monocular_image_visualization
  remote_stereo_image_visualization
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

install(TARGETS ${targets}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

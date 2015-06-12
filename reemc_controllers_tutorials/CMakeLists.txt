cmake_minimum_required(VERSION 2.8.3)
project(reemc_controllers_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  control_msgs
  sensor_msgs
  realtime_tools
  controller_interface
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/hello_controller.cpp
                                     src/force_torque_controller.cpp
                                     src/imu_controller.cpp
                                     src/joint_controller.cpp    
                                     src/combined_resource_controller.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

install(FILES reemc_tutorial_controller_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

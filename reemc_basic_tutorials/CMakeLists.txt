cmake_minimum_required(VERSION 2.8.3)
project(reemc_basic_tutorials)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

#############
## Install ##
#############

foreach(dir config launch)
  install(DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

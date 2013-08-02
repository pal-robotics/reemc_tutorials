cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(reach_pose src/reach_pose.cpp)
rosbuild_add_executable(asr_client src/asr_client.cpp)
rosbuild_add_executable(tts_client src/tts_client.cpp)

rosbuild_add_executable(look_to_point                        src/look_to_point.cpp)
rosbuild_add_executable(remote_monocular_image_visualization src/remote_monocular_image_visualization.cpp)
target_link_libraries(remote_monocular_image_visualization  pal_camera_client)
rosbuild_add_executable(remote_stereo_image_visualization src/remote_stereo_image_visualization.cpp)
target_link_libraries(remote_stereo_image_visualization  pal_camera_client)

rosbuild_add_executable(fullbody_person_2d_detection src/fullbody_person_2d_detection.cpp)
rosbuild_link_boost(fullbody_person_2d_detection signals)

rosbuild_add_executable(face_detection src/face_detection.cpp)
rosbuild_add_boost_directories()
rosbuild_link_boost(face_detection signals)

rosbuild_add_executable(face_enrollment src/face_enrollment.cpp)
rosbuild_add_executable(face_recognition src/face_recognition.cpp)


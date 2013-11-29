cmake_minimum_required(VERSION 2.8.3)
project(uvc_camera)

find_package(catkin REQUIRED COMPONENTS camera_info_manager image_transport nodelet roscpp sensor_msgs)

find_package(Boost REQUIRED COMPONENTS thread system)

find_package(PkgConfig REQUIRED)
pkg_check_modules(udev REQUIRED libudev)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nodelet_uvc_camera
  CATKIN_DEPENDS roscpp nodelet sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_library(xunit
  src/xunit/xunit_lib.c
  src/xunit/xunit_lib_cug.c
)
target_link_libraries(xunit ${udev_LIBRARIES})

add_library(nodelet_uvc_camera src/nodelets.cpp src/camera.cpp src/stereo.cpp src/uvc_cam.cpp)
target_link_libraries(nodelet_uvc_camera ${Boost_LIBRARIES} ${catkin_LIBRARIES} xunit)

add_executable(uvc_camera_node src/camera_node.cpp src/camera.cpp src/uvc_cam.cpp)
target_link_libraries(uvc_camera_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} xunit)

add_executable(uvc_stereo_node src/stereo_node.cpp src/stereo.cpp src/uvc_cam.cpp)
target_link_libraries(uvc_stereo_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} xunit)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS nodelet_uvc_camera uvc_camera_node uvc_stereo_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  nodelet_uvc_camera.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_uvc_camera.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

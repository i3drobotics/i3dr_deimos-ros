cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)
project(deimos)

find_package(catkin REQUIRED COMPONENTS camera_info_manager image_transport nodelet roscpp sensor_msgs)

find_package(Boost REQUIRED COMPONENTS thread system)

FIND_PACKAGE(PkgConfig)
PKG_CHECK_MODULES(PC_GLIB QUIET glib-2.0)

FIND_LIBRARY(
	GLIB_LIBRARIES
	NAMES glib-2.0
	HINTS ${PC_GLIB_LIBDIR}
	${PC_GLIB_LIBRARY_DIRS}
)

# Files in glib's main include path may include glibconfig.h, which,
# for some odd reason, is normally in $LIBDIR/glib-2.0/include.
GET_FILENAME_COMPONENT(_GLIB_LIBRARY_DIR ${GLIB_LIBRARIES} PATH)
FIND_PATH(
	GLIBCONFIG_INCLUDE_DIR
	NAMES glibconfig.h
	HINTS ${PC_LIBDIR} ${PC_LIBRARY_DIRS} ${_GLIB_LIBRARY_DIR}
	PATH_SUFFIXES glib-2.0/include
)

FIND_PATH(
	GLIB_INCLUDE_DIR
	NAMES glib.h
	HINTS ${PC_GLIB_INCLUDEDIR}
	${PC_GLIB_INCLUDE_DIRS}
	PATH_SUFFIXES glib-2.0
)

SET(GLIB_INCLUDE_DIRS ${GLIB_INCLUDE_DIR} ${GLIBCONFIG_INCLUDE_DIR})

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS roscpp nodelet sensor_msgs
#  DEPENDS system_lib
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/src/xunit
	${GLIB_INCLUDE_DIRS}
)

add_library(xunit
  src/xunit/xunit_lib.c
  src/xunit/xunit_lib_cug.c
)

add_library(xunitTara
  src/xunit/xunit_lib_tara.h
  src/xunit/xunit_lib_tara.cpp
) 

target_link_libraries(xunitTara ${udev_LIBRARIES} ${GLIB2_LIBRARIES})  
target_link_libraries(xunit ${udev_LIBRARIES})

add_executable(deimos_node src/deimos_node.cpp src/deimos_ros.cpp src/uvc_cam.cpp)
target_link_libraries(deimos_node ${Boost_LIBRARIES} ${catkin_LIBRARIES} xunitTara ${GLIB2_LIBRARIES} -lv4l2 -lyaml-cpp -ludev -lglib-2.0)

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

install(TARGETS deimos_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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

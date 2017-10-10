cmake_minimum_required(VERSION 2.8.3)
project(uwsimbenchmarks)

find_package(catkin REQUIRED COMPONENTS
  std_srvs
  pcl_ros
  topic_tools 
  rospy
  std_msgs
  message_generation
  uwsim
)

find_package(uwsim_osgocean REQUIRED)
find_package(uwsim_osgworks REQUIRED)
find_package(uwsim_osgbullet REQUIRED)
find_package(uwsim_bullet REQUIRED)
find_package(osg_utils REQUIRED)
find_package(osg_interactive_markers REQUIRED)
find_package(osg_markers REQUIRED)

add_service_files(
  FILES
  GTpublish.srv
)

generate_messages(
  DEPENDENCIES
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_srvs
    pcl_ros
    topic_tools
    uwsim
  DEPENDS
    roscpp
    message_runtime
    muparser
    osg_utils
    osg_markers
    osg_interactive_markers
    uwsim_bullet
    uwsim_osgocean
    uwsim_osgworks
)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;{CMAKE_MODULE_PATH}")
SET(UWSIMBENCHMARKS_ROOT_PATH "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}")

CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/src/BenchmarksConfig.h.cmake
  ${PROJECT_SOURCE_DIR}/src/BenchmarksConfig.h
)

FIND_PACKAGE ( OpenSceneGraph COMPONENTS osgSim osgUtil osgDB osgFX osgGA osgTerrain osgViewer osgText osgWidget osg)


find_package(PkgConfig)
pkg_check_modules(LIBXML++ REQUIRED libxml++-2.6)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

INCLUDE_DIRECTORIES( ${OPENSCENEGRAPH_INCLUDE_DIRS} ${LIBXML++_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${OSGOCEAN_INCLUDE_DIRS} ${OSGBULLET_INCLUDE_DIR} ${BULLET_INCLUDE_DIRS} ${OSGWORKS_INCLUDE_DIR})

# this disables the warnings that come mostly from osg
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-qualifiers")
# Needed for kinetic
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable(uwsimbenchmarks src/main.cpp
src/Measures.cpp
src/SceneUpdater.cpp
src/Trigger.cpp
src/BenchmarkXMLParser.cpp
src/Benchmark.cpp
src/BenchmarkROSInterfaces.cpp
src/Current.cpp
src/BagPlayer.cpp
src/FogAdderBagPlayer.cpp)


TARGET_LINK_LIBRARIES(uwsimbenchmarks   ${OSGOCEAN_LIBRARIES} ${PCL_LIBRARIES} ${catkin_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES}  ${BULLET_LIBRARIES} ${OSGBULLET_LIBRARIES} ${OSGWORKS_LIBRARIES} ${OSGUTILS_LIBRARIES} ${OSGINTERACTIVEMARKERS_LIBRARIES} ${OSGMARKERS_LIBRARIES}  -lmuparser GLU)

install(
  TARGETS
    uwsimbenchmarks
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY data/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)

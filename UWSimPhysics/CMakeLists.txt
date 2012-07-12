cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;{CMAKE_MODULE_PATH}")
set(BUILD_ROS_INTERFACES 1)

FIND_PACKAGE ( OpenSceneGraph COMPONENTS osgSim osgUtil osgDB osgFX osgGA osgTerrain osgViewer osgText osgWidget osgManipulator osg)

pkg_check_modules(LIBXML++ REQUIRED libxml++-2.6)

INCLUDE_DIRECTORIES(${OPENSCENEGRAPH_INCLUDE_DIRS} ${LIBXML++_INCLUDE_DIRS})
LINK_DIRECTORIES(${LIBXML++_LIBRARY_DIRS})

# this disables the warnings that come mostly from osg
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-ignored-qualifiers")


rosbuild_add_library(uwsimphysics
src/BulletPhysics.cpp
src/PhysicsBuilder.cpp)

rosbuild_add_executable(UWSimPhysics src/main.cpp)

TARGET_LINK_LIBRARIES(UWSimPhysics uwsimphysics ${OPENSCENEGRAPH_LIBRARIES} ${LIBXML++_LIBRARIES} GLU)

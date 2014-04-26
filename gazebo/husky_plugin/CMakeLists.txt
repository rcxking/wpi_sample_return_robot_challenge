cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
	pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
##cmake_minimum_required(VERSION 2.8.3)
##project(husky_gazebo_plugins)

##find_package(catkin REQUIRED COMPONENTS
  ##gazebo_ros
##)

##find_package(gazebo REQUIRED)

##include_directories(include ${catkin_INCLUE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormate_INCLUDE_DIRS})
# Set the build type. Options are:
# Coverage : w/ debug symbols, w/o optimization, w/ code-coverage
# Debug : w/ debug symbols, w/o optimization
# Release : w/o debug symbols, w/ optimization
# RelWithDebInfo : w/ debug symbols, w/ optimization
# MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()

#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
  
add_library(${PROJECT_NAME} SHARED src/husky_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_libraries})

#rosbuild_link_boost(${PROJECT_NAME} thread)

##catkin_package(
##	DEPENDS
##		gazebo_ros
##	CATKIN_DEPENDS
##	INCLUDE_DIRS
##	LIBRARIES
##)

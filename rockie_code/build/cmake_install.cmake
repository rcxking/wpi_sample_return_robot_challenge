# Install script for directory: /home/will/Code/wpi-sample-return-robot-challenge/rockie_code/src

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install/.catkin")
FILE(INSTALL DESTINATION "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install" TYPE FILE FILES "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/catkin_generated/installspace/.catkin")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install/_setup_util.py")
FILE(INSTALL DESTINATION "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install" TYPE PROGRAM FILES "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/catkin_generated/installspace/_setup_util.py")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install/env.sh")
FILE(INSTALL DESTINATION "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install" TYPE PROGRAM FILES "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/catkin_generated/installspace/env.sh")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install/setup.bash")
FILE(INSTALL DESTINATION "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install" TYPE FILE FILES "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/catkin_generated/installspace/setup.bash")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install/setup.sh")
FILE(INSTALL DESTINATION "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install" TYPE FILE FILES "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/catkin_generated/installspace/setup.sh")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install/setup.zsh")
FILE(INSTALL DESTINATION "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install" TYPE FILE FILES "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/catkin_generated/installspace/setup.zsh")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CPACK_ABSOLUTE_DESTINATION_FILES
   "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install/.rosinstall")
FILE(INSTALL DESTINATION "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/install" TYPE FILE FILES "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/catkin_generated/installspace/.rosinstall")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/catkin/profile.d" TYPE FILE FILES "/opt/ros/hydro/share/catkin/cmake/env-hooks/05.catkin_make.bash")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/etc/catkin/profile.d" TYPE FILE FILES "/opt/ros/hydro/share/catkin/cmake/env-hooks/05.catkin_make_isolated.bash")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/gtest/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/libviso2/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/rockie_executive/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/slam_db/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/stereo_feature_identifier/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/stereo_feature_matcher/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/stereo_feature_triangulator/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/stereo_graph_manager/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/stereo_historian/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/stereo_localizer/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/stereo_obstacle_detection/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/stereo_publisher/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/stereo_wpi_feature_finder/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/tf_robot/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/viso2/cmake_install.cmake")
  INCLUDE("/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/viso2_ros/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/build/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)

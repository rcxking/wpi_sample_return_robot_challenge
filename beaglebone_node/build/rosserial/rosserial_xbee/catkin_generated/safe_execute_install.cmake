execute_process(COMMAND "/home/bryant/wpi_sample_return_robot_challenge/beaglebone_node/build/rosserial/rosserial_xbee/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/bryant/wpi_sample_return_robot_challenge/beaglebone_node/build/rosserial/rosserial_xbee/catkin_generated/python_distutils_install.sh) returned error code ")
endif()

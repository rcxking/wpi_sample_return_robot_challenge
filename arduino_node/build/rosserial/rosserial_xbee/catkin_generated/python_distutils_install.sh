#!/bin/sh -x

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

cd "/home/bryant/wpi_sample_return_robot_challenge/arduino_node/src/rosserial/rosserial_xbee"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/bryant/wpi_sample_return_robot_challenge/arduino_node/install/lib/python2.7/dist-packages:/home/bryant/wpi_sample_return_robot_challenge/arduino_node/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/bryant/wpi_sample_return_robot_challenge/arduino_node/build" \
    "/usr/bin/python" \
    "/home/bryant/wpi_sample_return_robot_challenge/arduino_node/src/rosserial/rosserial_xbee/setup.py" \
    build --build-base "/home/bryant/wpi_sample_return_robot_challenge/arduino_node/build/rosserial/rosserial_xbee" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/bryant/wpi_sample_return_robot_challenge/arduino_node/install" --install-scripts="/home/bryant/wpi_sample_return_robot_challenge/arduino_node/install/bin"

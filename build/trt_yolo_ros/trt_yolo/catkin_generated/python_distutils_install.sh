#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/uwfsae/driverless_ws/src/trt_yolo_ros/trt_yolo"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/uwfsae/driverless_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/uwfsae/driverless_ws/install/lib/python2.7/dist-packages:/home/uwfsae/driverless_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/uwfsae/driverless_ws/build" \
    "/usr/bin/python2" \
    "/home/uwfsae/driverless_ws/src/trt_yolo_ros/trt_yolo/setup.py" \
     \
    build --build-base "/home/uwfsae/driverless_ws/build/trt_yolo_ros/trt_yolo" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/uwfsae/driverless_ws/install" --install-scripts="/home/uwfsae/driverless_ws/install/bin"

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

echo_and_run cd "/home/uriel/ros-darwin/src/darwin_gazebo"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/uriel/ros-darwin/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/uriel/ros-darwin/install/lib/python3/dist-packages:/home/uriel/ros-darwin/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/uriel/ros-darwin/build" \
    "/usr/bin/python3" \
    "/home/uriel/ros-darwin/src/darwin_gazebo/setup.py" \
     \
    build --build-base "/home/uriel/ros-darwin/build/darwin_gazebo" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/uriel/ros-darwin/install" --install-scripts="/home/uriel/ros-darwin/install/bin"

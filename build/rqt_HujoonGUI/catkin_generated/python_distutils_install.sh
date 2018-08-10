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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ckim/HujoonROS/src/rqt_HujoonGUI"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ckim/HujoonROS/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ckim/HujoonROS/install/lib/python2.7/dist-packages:/home/ckim/HujoonROS/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ckim/HujoonROS/build" \
    "/usr/bin/python" \
    "/home/ckim/HujoonROS/src/rqt_HujoonGUI/setup.py" \
    build --build-base "/home/ckim/HujoonROS/build/rqt_HujoonGUI" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/ckim/HujoonROS/install" --install-scripts="/home/ckim/HujoonROS/install/bin"

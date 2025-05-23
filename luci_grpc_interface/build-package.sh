#!/bin/bash

rm -rf debian

# Note it is assumed that this file is at the root dir of the package, that is its next to package.xml file
source /opt/ros/humble/setup.bash

# Setup .deb file rules
rosdep init
rosdep update
bloom-generate rosdebian

# Build deb file
fakeroot debian/rules binary

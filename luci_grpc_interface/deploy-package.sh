#!/bin/bash

# NOTE: THIS SHOULD NOT BE RAN MANAULLY, THIS IS A SUPPORTING FILE FOR THE GITHUB ACTIONS

# NOTE: It is assumed that this file is at the root dir of the package, that is its next to package.xml file

# Get package name that will be on dir above this file
PACKAGE_PATH="../*.deb"
OS_VERSION="jammy"
COMPONENT="private"
ARCHITECTURE="amd64"

FILE=$(basename $PACKAGE_PATH)
echo $FILE
jf rt u --deb=$OS_VERSION/$COMPONENT/$ARCHITECTURE $PACKAGE_PATH ros2-sdk-packages/$FILE 

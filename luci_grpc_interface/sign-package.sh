#!/bin/bash

# NOTE: it is assumed that this file is at the root dir of the package, that is its next to package.xml file

PACKAGE_PATH="../*.deb"
FILE=$(basename $PACKAGE_PATH)
echo "FILE: $FILE"

# List key being used for action debugging, should match keyid output from import step
echo "SIGNING ID ${KEY_ID}"

# Sign the package
dpkg-sig -k ${KEY_ID} --sign builder ../$FILE --verbose

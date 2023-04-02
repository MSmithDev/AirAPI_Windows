#!/bin/bash

# build vars
PROJ_NAME="AirAPI_Mac"
PROJ_LIB_NAME="lib$PROJ_NAME.a"
PROJ_LIB_HEADER_FILENAME="$PROJ_NAME.h"
PROJ_ROOT_DIR="$(pwd)"
BUILD_MAC_DIR="$PROJ_ROOT_DIR/build/mac"
BUILD_MAC_LIB_DIR=$BUILD_MAC_DIR/lib
BUILD_MAC_INCLUDE_DIR=$BUILD_MAC_DIR/include

# setup dirs
mkdir -p $BUILD_MAC_DIR
mkdir -p $BUILD_MAC_LIB_DIR
mkdir -p $BUILD_MAC_INCLUDE_DIR

# copy
cp "$PROJ_ROOT_DIR/platforms/mac/AirAPI_Mac.h" "$BUILD_MAC_INCLUDE_DIR/AirAPI_Mac.h" &&

# run cmake build to release
cmake -B $BUILD_MAC_DIR -DCMAKE_BUILD_TYPE=Release && cmake --build $BUILD_MAC_DIR

# copy output to lib dir
mv "$BUILD_MAC_DIR/libAirAPI_Mac.a" $BUILD_MAC_LIB_DIR

# build new output paths
LIB_PATH="$BUILD_MAC_LIB_DIR/$PROJ_LIB_NAME"
HEADER_PATH="$BUILD_MAC_INCLUDE_DIR/$PROJ_LIB_HEADER_FILENAME"

# check if successful
if [ -f "$LIB_PATH" ] && [ -f "$HEADER_PATH" ]; then
    # report success
    printf "\n\nmacOS build script finished, the compiled framework paths are:\n\n"
    printf "header path:\n$HEADER_PATH\n\n"
    printf "library path:\n$LIB_PATH\n\n"
else
    printf "\nERROR: the script did not finish building successfully.\n\nSee logs above.\n"
fi
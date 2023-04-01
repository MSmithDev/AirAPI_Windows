# TODO: post-draft pr, remove bg / investigation comments and links
# also-- see if all is required and drop anything not mandatory
cmake_minimum_required(VERSION 3.17)

# macos build config
# project config
project(AirAPI_Mac)

# compiler config
SET(CMAKE_CXX_COMPILER             "/usr/bin/clang++")
SET(CMAKE_CXX_FLAGS                "-Wall")
SET(CMAKE_CXX_FLAGS_DEBUG          "-g")
SET(CMAKE_CXX_FLAGS_MINSIZEREL     "-Os -DNDEBUG")
SET(CMAKE_CXX_FLAGS_RELEASE        "-O4 -DNDEBUG")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_STANDARD 17)

# deps and sources config
set(HIDAPI_HEADER_PATH ${HIDAPI_INCLUDE_DIR}) # sourced from env @ cmake --build
set(MAC_HEADER_PATH "${CMAKE_CURRENT_SOURCE_DIR}/platforms/mac")
set(SOURCES ./AirAPI_Windows.cpp "${HIDAPI_HEADER_PATH}" "${MAC_HEADER_PATH}")

# declare a lib, not executable
add_library(AirAPI_Mac STATIC ${SOURCES})

# set library directory
set(HIDAPI_INCLUDE_DIR "$HIDAPI_INCLUDE_DIR")

# include dirs
target_include_directories(AirAPI_Mac PUBLIC "${HIDAPI_INCLUDE_DIR}")
target_include_directories(AirAPI_Mac INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

# link specific object files we need
target_link_libraries(AirAPI_Mac ${FUSION_LIB_DIR}/libFusion.a)
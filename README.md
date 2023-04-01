# AirAPI_Windows

This is a work in progress driver for the Nreal Air for native windows support without nebula. The current configuration for fusion has slight drift and needs to be tuned but is functional. This project is a colaberataive effort by the community to provide open access to the glasses.

This is made possible by these wonderful people:<br>
https://github.com/abls <br>
https://github.com/edwatt <br>
https://github.com/jakedowns

## Precompiled
To use the precompiled [AirAPI_Windows.dll](https://github.com/MSmithDev/AirAPI_Windows/releases) you also need to include hidapi.dll available [here](https://github.com/libusb/hidapi/releases). 

[Here](https://github.com/MSmithDev/AirAPI_Windows/wiki/Using-with-Unity) is a demo in the wiki for use with unity scripts.

[Here](https://github.com/MSmithDev/AirPoseUnityDemo) is a Unity demo using the XR plugin with the Airs setup as a headset.

# Building from source

## Windows

### Getting hidapi
Get the latest hidapi-win.zip from [here](https://github.com/libusb/hidapi/releases).

unzip hidapi-win.zip to "deps" folder ("deps/hidapi-win").



### Build Fusion
#### Clone this project
Goto project directory
```
cd AirAPI_Windows
```
Init sub module
```
git submodule init
```
update the module
```
git submodule update
```


cd to "deps/Fusion/Fusion"

Run cmake
```
cmake .
```

Open the Project.sln and build the project <br>
You should have a "deps/Fusion/Fusion/Release/Fusion.lib" file.
#### Build AirAPI_Windows DLL 
Open AirAPI_Windows.sln and make sure "Release" and "x64" are set and build.

## MacOS (Beta)

Features:

- C++ library compilation and linking for macOS
- Compatible with **Swift** for macOS app development for **BOTH Intel AND Arm based Macs**
- Same interfaces and features as the Windows version
- Compatible with the latest macOS releases
- Builds and runs alongside the Windows version with no additional configuration steps

Requirements:

- macOS v13+ (ventura)
- Same dependency requirements as Windows, but you need to install / link them differently (windows instructions don't apply to the macOS build process)
    - `Fusion`
    - `hdiapi`


Known issues:

- the brightness API doesn't seem to produce any data

    - looks like an underlying issue in the C++, needs investigation


### Preparing to build from source

Make sure you have the tools to build this project for macOS Ventura:

- Install [cmake](https://formulae.brew.sh/formula/cmake) from Homebrew

- Make sure you have Apple’s developer tools installed:
    - Get the latest version of  Xcode [from the Mac App Store](https://apps.apple.com/us/app/xcode/id497799835?mt=12)
    - Make sure you have the latest toolchain by running: `xcode-select --install`

- 

Build the dependencies locally for macOS from source.


### Compile Fusion for macOS

You'll need to build `Fusion` locally for macOS from source:

- `cd` to the root of this repository

- run `git submodule update`

- `cd` from project root to the `Fusion` package with `cd ./deps/Fusion/Fusion`


From here the instructions are different from the Windows build becuase you need both a header file AND a compiled c library (Fusion.h and libfusion.a) for macOS:

- create build output some folders to organize cmake output

    - run `mkdir -p ./build/lib && mkdir ./build/include`

- run `cmake -B build -DCMAKE_BUILD_TYPE=Release`

    - cmake artifacts will be created within the `build` directory

- run `cmake --build build`

    - this produces a static library `libFusion.a` within `build`

- organize cmake artifacts into a conventional c++ dir structure:

    - run `cp ./*.h ./build/include` to copy the library’s c++ headers
    - run `cp ./build/libFusion.a ./build/lib` to move the static library to the `./build/lib`


if you run `tree -I 'CMakeFiles' ./build` the directory structure should like this:
<br>

```bash
# install tree if you don’t have it:
# brew install tree

$ tree -I 'CMakeFiles' ./build

./build
├── CMakeCache.txt
├── Makefile
├── cmake_install.cmake
├── include
│   ├── Fusion.h
│   ├── FusionAhrs.h
│   ├── FusionAxes.h
│   ├── FusionCalibration.h
│   ├── FusionCompass.h
│   ├── FusionMath.h
│   └── FusionOffset.h
├── lib
│   └── libFusion.a
└── libFusion.a

3 directories, 12 files
```
<br>


#### Export Fusion paths


You will need to link `AirAPI_Mac` the the `Fusion` library for macOS later on.

We suggest you export those paths by running the following from `deps/Fusion/Fusion`:


```bash
export FUSION_BUILD_DIR="$(pwd)"
export FUSION_LIB_DIR="$FUSION_BUILD_DIR/lib"
export FUSION_INCLUDE_DIR="$FUSION_BUILD_DIR/include"
```

the output of both:

- `echo $FUSION_LIB_DIR` 

- `echo $FUSION_INCLUDE_DIR` 

should display the paths you will need later when compiling `AirAPI_Mac` with cmake

<br>
Alternatively, you can also installl `Fusion` for macOS by [building the project from source](https://github.com/xioTechnologies/Fusion) directly.
<br><br>


### Compile hidapi for macOS

Installing `hidapi` with Homebrew is a bit easier:

- run: `brew update` 

- run: `brew install hidapi`

The installation output should live in `/usr/local/Cellar/hidapi` by default.

(You can confirm the istallation location at any time by running `brew info hidapi`, which will list the installation path)
<br>


### Export hidapi paths


Assuming that you used `brew` to install `hidapi`, export the paths for `hidapi`:

```bash
export HIDAPI_ROOT_DIR="/usr/local/Cellar/hidapi/0.13.1"
export HIDAPI_LIB_DIR="$HIDAPI_ROOT_DIR/lib"
export HIDAPI_INCLUDE_DIR="$HIDAPI_ROOT_DIR/include"
```
*Note: 
version 0.13.1 was the latest at the time of writing this. run **brew info hidapi** to verify your current version*


the output of both:

- `echo $HIDAPI_LIB_DIR` 

- `echo $HIDAPI_INCLUDE_DIR` 

should display the paths you will need later when compiling `AirAPI_Mac` with cmake


<br>
Alternatively, you can also installl `hidapi` for macOS by [building the project from source](https://github.com/libusb/hidapi) directly.
<br><br>


## Building a macOS compatible c++ library
<br>
Assuming you have all of the tools and dependencies above installed:

- Clone this repository

- `cd` to the root of the reopsitory


**Required**: to run the included `build-mac.sh` script, these environment variables *must* be exported in the current terminal (details above):

    - `$FUSION_LIB_DIR`
    - `$FUSION_INCLUDE_DIR`
    - `$HIDAPI_LIB_DIR`
    - `$HIDAPI_INCLUDE_DIR`


### Run the build-mac script


From the root of this repo:

- `zsh ./platforms/mac/build-mac.sh`


After a fairly quick build process, you should see the following:

```bash
macOS build complete.

header path:
(your local path)/AirAPI_Windows/build/mac/include/AirAPI_Mac.h

library path:
(your local path)/AirAPI_Windows/build/mac/lib/libAirAPI_Mac.a
```


If you see this ^^ message=, congrats, you’ve built `AirAPI_Mac` successfully.


## Building a compatible Swift Framework for macOS

Although c++ and Swift are technically interoperable, using a c++ library with Swift requires an Objective-C implementation layer in between.

### (INSTRUCTIONS AND EXAMPLE PROJECT COMING NEXT)


#### Disclaimer about this proejct

(and open-source software generally)

As always, please understand the risks including hardware / software data loss or damage, security issues, etc. when working with code from the internet. We're doing our best, but the maintainers of this repository (or whoever you got it from), collaborators, etc make no guarantees, make no warranties, and are not responsible for anything that happens as a result of this code or any of its dependencies. You are fully, 100% responsible for anything you do with this work, and by simply being in possession of this code (or any derivitive work that includes, depends on, or references this project) that you are on your own and fully liable for whatever happens next. 

In general: if you don't know what it does, don't download or run it.
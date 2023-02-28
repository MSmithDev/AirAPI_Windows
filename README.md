# AirAPI_Windows

This is a work in progress driver for the Nreal Air for native windows support without nebula. The current configuration for fusion has slight drift and needs to be tuned but is functional.

## Getting hidapi
Get the latest hidapi-win.zip from [here](https://github.com/libusb/hidapi/releases).

unzip hidapi-win.zip to "deps" folder ("deps/hidapi-win").

## Build Fusion
cd to "deps/Fusion/Fusion"

Run cmake
```
cmake .
```

Open the Project.sln and build the project <br>
You should have a "deps/Fusion/Fusion/Release/Fusion.lib" file.

## Build AirAPI_Windows DLL

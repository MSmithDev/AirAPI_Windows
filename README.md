# AirAPI_Windows

This is a work in progress driver for the Nreal Air for native windows support without nebula. The current configuration for fusion has slight drift and needs to be tuned but is functional. This project is a colaberataive effort by the community to provide open access to the glasses.

This is made possible by these wonderful people:<br>
https://github.com/abls <br>
https://github.com/edwatt <br>
https://github.com/jakedowns

Feel free to support me with a coffee.
https://www.buymeacoffee.com/msmithdev

## Precompiled
To use the precompiled [AirAPI_Windows.dll](https://github.com/MSmithDev/AirAPI_Windows/releases) you also need to include hidapi.dll available [here](https://github.com/libusb/hidapi/releases). 

[Here](https://github.com/MSmithDev/AirAPI_Windows/wiki/Using-with-Unity) is a demo in the wiki for use with unity scripts.

[Here](https://github.com/MSmithDev/AirPoseUnityDemo) is a Unity demo using the XR plugin with the Airs setup as a headset.

# Building from source

## Getting hidapi
Get the latest hidapi-win.zip from [here](https://github.com/libusb/hidapi/releases).

unzip hidapi-win.zip to "deps" folder ("deps/hidapi-win").



## Build Fusion
### Clone this project
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
### Build AirAPI_Windows DLL 
Open AirAPI_Windows.sln and make sure "Release" and "x64" are set and build.

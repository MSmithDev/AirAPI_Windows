// TODO: post-draft pr, remove bg / investigation comments and links

// macOS compatibility
#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))

#ifdef AIRAPI_EXPORTS
#define AIR_API __attribute__((visibility("default")))
#else
#define AIR_API
#endif

#else

#pragma once

#ifdef AIRAPI_EXPORTS
#define AIR_API __declspec(dllexport)
#else
#define AIR_API __declspec(dllimport)
#endif

#endif

//Function to start connection to Air
extern "C" AIR_API int StartConnection();

//Function to stop connection to Air
extern "C" AIR_API int StopConnection();

//Function to get quaternion
extern "C" AIR_API float* GetQuaternion();

//Function to get euler
extern "C" AIR_API float* GetEuler();

//Function to get brightness
extern "C" AIR_API int GetBrightness();
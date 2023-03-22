#pragma once

#ifdef AIRAPI_EXPORTS
#define AIR_API __declspec(dllexport)
#else
#define AIR_API __declspec(dllimport)
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

//Function to set fusion gain
extern "C" AIR_API int SetFusionGain(float);

//Function to set accel rejection
extern "C" AIR_API int SetFusionAccelRejection(float);

//Function to set mag rejection 
extern "C" AIR_API int SetFusionMagRejection(float);

//Function to set mag rejection 
extern "C" AIR_API int SetFusionRejectTimeout(unsigned int);
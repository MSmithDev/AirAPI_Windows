// AirAPI_Mac.h
//
// Created by GigabiteLabs
// Swift Version: 5.0
// Copyright © 2023 GigabiteLabs. All rights reserved.
//

#pragma once

#define AIRAPI_MAC __attribute__((visibility("public")))

#if __cplusplus
extern "C" {
#endif

//Function to start connection to Air
int StartConnection(void);
//Function to stop connection to Air
int StopConnection(void);
//Function to get quaternion
float * GetQuaternion(void);
//Function to get euler
float * GetEuler(void);
//Function to get brightness
int GetBrightness(void);

#if __cplusplus
}   // Extern C
#endif

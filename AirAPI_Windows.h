#pragma once

#ifdef AIRAPI_EXPORTS
#define AIR_API __declspec(dllexport)
#else
#define AIR_API __declspec(dllimport)
#endif

#include <cstdint>

//Function to start connection to Air
extern "C" AIR_API int StartConnection();

//Function to stop connection to Air
extern "C" AIR_API int StopConnection();

//Function to get quaternion
extern "C" AIR_API float* GetQuaternion();

//Function to get euler
extern "C" AIR_API float* GetEuler();

//Function to get rawGyro
extern "C" AIR_API float* GetRawGyro();

//Function to get rawAccel
extern "C" AIR_API float* GetRawAccel();

//Function to get rawMag
extern "C" AIR_API float* GetRawMag();

//funtion to get timestamp
extern "C" AIR_API uint64_t GetAirTimestamp();

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



#pragma pack(push, 1)
struct AirDataPacket {
	
	uint8_t signature[2];
	uint8_t temperature[2];
	uint64_t timestamp;
	uint8_t angular_multiplier[2];
	uint8_t angular_divisor[4];
	uint8_t angular_velocity_x[3];
	uint8_t angular_velocity_y[3];
	uint8_t angular_velocity_z[3];
	uint8_t acceleration_multiplier[2];
	uint8_t acceleration_divisor[4];
	uint8_t acceleration_x[3];
	uint8_t acceleration_y[3];
	uint8_t acceleration_z[3];
	uint8_t magnetic_multiplier[2];
	uint8_t magnetic_divisor[4];
	uint8_t magnetic_x[2];
	uint8_t magnetic_y[2];
	uint8_t magnetic_z[2];
	uint32_t checksum;
	uint8_t _padding[7];
};
#pragma pack(pop)
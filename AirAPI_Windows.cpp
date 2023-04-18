#include "pch.h"
#include "AirAPI_Windows.h"
#include "deps/hidapi-win/include/hidapi.h"
#include "deps/Fusion/Fusion/Fusion.h"
#include <iostream>
#include <mutex>
#include <array>
#include <cstdint>
#include <vector>
//Air USB VID and PID
#define AIR_VID 0x3318
#define AIR_PID 0x0424

//Is Tracking
bool g_isTracking = false;

//Is Listening
bool g_isListening = false;

// ticks are in nanoseconds, 1000 Hz packets
#define TICK_LEN (1.0f / 1E9f)

// based on 24bit signed int w/ FSR = +/-2000 dps, datasheet option
#define GYRO_SCALAR (1.0f / 8388608.0f * 2000.0f)

// based on 24bit signed int w/ FSR = +/-16 g, datasheet option
#define ACCEL_SCALAR (1.0f / 8388608.0f * 16.0f)

static int rows, cols;
static FusionEuler euler;
static FusionVector earth;
static FusionQuaternion qt;

HANDLE trackThread;
HANDLE listenThread;

hid_device* device;

hid_device* device4;

#define SAMPLE_RATE (1000) // replace this with actual sample rate

FusionAhrs ahrs;

// Set AHRS algorithm settings
FusionAhrsSettings settings = {
		.gain = 0.5f,
		.accelerationRejection = 10.0f,
		.magneticRejection = 20.0f,
		.rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
};

std::mutex mtx;
std::mutex it4;

typedef struct {
	uint64_t tick;
	int32_t ang_vel[3];
	int32_t accel[3];
	int16_t mag[3];
} air_sample;


static int32_t pack32bit_signed(const uint8_t* data) {
	uint32_t unsigned_value = (data[0]) | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
	return ((int32_t)unsigned_value);
}

static int32_t pack24bit_signed(const uint8_t* data) {
	uint32_t unsigned_value = (data[0]) | (data[1] << 8) | (data[2] << 16);
	if ((data[2] & 0x80) != 0) unsigned_value |= (0xFF << 24);
	return ((int32_t)unsigned_value);
}

static int16_t pack16bit_signed(const uint8_t* data) {
	uint16_t unsigned_value = (data[0]) | (data[1] << 8);
	return (int16_t)unsigned_value;
}

static int32_t pack32bit_signed_swap(const uint8_t* data) {
	uint32_t unsigned_value = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]);
	return ((int32_t)unsigned_value);
}

static int16_t pack16bit_signed_swap(const uint8_t* data) {
	uint16_t unsigned_value = (data[0] << 8) | (data[1]);
	return (int16_t)unsigned_value;
}


static int parse_report(AirDataPacket* packet, int size, air_sample* out_sample) {


	const uint64_t timestamp = packet->timestamp;
	std::cout << "TS: " << timestamp << std::endl;
	out_sample->tick = timestamp;

	int32_t vel_m = pack16bit_signed(packet->angular_multiplier);
	int32_t vel_d = pack32bit_signed(packet->angular_divisor);

	int32_t vel_x = pack24bit_signed(packet->angular_velocity_x);
	int32_t vel_y = pack24bit_signed(packet->angular_velocity_y);
	int32_t vel_z = pack24bit_signed(packet->angular_velocity_z);

	out_sample->ang_vel[0] = (float)vel_x * vel_m / vel_d;
	out_sample->ang_vel[1] = (float)vel_y * vel_m / vel_d;
	out_sample->ang_vel[2] = (float)vel_z * vel_m / vel_d;

	int32_t accel_m = pack16bit_signed(packet->acceleration_multiplier);
	int32_t accel_d = pack32bit_signed(packet->acceleration_divisor);

	int32_t accel_x = pack24bit_signed(packet->acceleration_x);
	int32_t accel_y = pack24bit_signed(packet->acceleration_y);
	int32_t accel_z = pack24bit_signed(packet->acceleration_z);

	out_sample->accel[0] = (float)accel_x * accel_m / accel_d;
	out_sample->accel[1] = (float)accel_y * accel_m / accel_d;
	out_sample->accel[2] = (float)accel_z * accel_m / accel_d;

	int32_t magnet_m = pack16bit_signed_swap(packet->magnetic_multiplier);
	int32_t magnet_d = pack32bit_signed_swap(packet->magnetic_divisor);

	int16_t magnet_x = pack16bit_signed_swap(packet->magnetic_x);
	int16_t magnet_y = pack16bit_signed_swap(packet->magnetic_y);
	int16_t magnet_z = pack16bit_signed_swap(packet->magnetic_z);

	out_sample->mag[0] = (float)magnet_x * magnet_m / magnet_d;
	out_sample->mag[1] = (float)magnet_y * magnet_m / magnet_d;
	out_sample->mag[2] = (float)magnet_z * magnet_m / magnet_d;


	return 1;
}



static void
process_ang_vel(const int32_t in_ang_vel[3], float out_vec[])
{

	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_ang_vel[0]) * -1.0f * GYRO_SCALAR;
	out_vec[1] = (float)(in_ang_vel[1]) * GYRO_SCALAR;
	out_vec[2] = (float)(in_ang_vel[2]) * GYRO_SCALAR;
}

static void
process_accel(const int32_t in_accel[3], float out_vec[])
{
	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_accel[0]) * ACCEL_SCALAR;
	out_vec[1] = (float)(in_accel[1]) * ACCEL_SCALAR;
	out_vec[2] = (float)(in_accel[2]) * ACCEL_SCALAR;

}


static hid_device*
open_device()
{
	struct hid_device_info* devs = hid_enumerate(AIR_VID, AIR_PID);
	struct hid_device_info* cur_dev = devs;
	hid_device* device = NULL;

	while (devs) {
		if (cur_dev->interface_number == 3) {
			device = hid_open_path(cur_dev->path);
			std::cout << "Interface 3 bound" << std::endl;
			break;
		}

		cur_dev = cur_dev->next;
	}

	hid_free_enumeration(devs);
	return device;
}

static hid_device*
open_device4()
{
	struct hid_device_info* devs = hid_enumerate(AIR_VID, AIR_PID);
	struct hid_device_info* cur_dev = devs;
	hid_device* device = NULL;

	while (devs) {
		if (cur_dev->interface_number == 4) {
			device = hid_open_path(cur_dev->path);
			std::cout << "Interface 4 bound" << std::endl;
			break;
		}

		cur_dev = cur_dev->next;
	}

	hid_free_enumeration(devs);
	return device;
}

struct ThreadParams {
	hid_device* device;
};
static float ang_vel[3] = {};
static float accel_vec[3] = {};
static float mag_vec[3] = {};
static uint64_t airTimestamp;



DWORD WINAPI track(LPVOID lpParam) {

	//Thread to handle tracking
	//unsigned char buffer[64] = {};
	AirDataPacket buffer;
	memset(&buffer, 0, sizeof(AirDataPacket));

	uint64_t last_sample_tick = 0;
	air_sample sample = {};
	ThreadParams* params = static_cast<ThreadParams*>(lpParam);
	//static FusionVector ang_vel = {}, accel_vec = {};



	// Define calibration (replace with actual calibration data if available)
	const FusionMatrix gyroscopeMisalignment = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
	const FusionVector gyroscopeSensitivity = { 1.0f, 1.0f, 1.0f };
	const FusionVector gyroscopeOffset = { 0.0f, 0.0f, 0.0f };
	const FusionMatrix accelerometerMisalignment = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
	const FusionVector accelerometerSensitivity = { 1.0f, 1.0f, 1.0f };
	const FusionVector accelerometerOffset = { 0.0f, 0.0f, 0.0f };
	const FusionMatrix softIronMatrix = { 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
	const FusionVector hardIronOffset = { 0.0f, 0.0f, 0.0f };


	// Initialise algorithms
	FusionOffset offset;


	FusionOffsetInitialise(&offset, SAMPLE_RATE);
	mtx.lock();
	FusionAhrsInitialise(&ahrs);
	FusionAhrsSetSettings(&ahrs, &settings);
	mtx.unlock();


	while (g_isTracking) {


		try {
			// code that might throw an exception
			int res = hid_read(device, (uint8_t*)&buffer, sizeof(buffer));
			if (res < 0) {
				break;
			}
		}
		catch (const std::exception& e) {
			// handle the exception
			std::cerr << e.what();
		}


		//parse
		parse_report(&buffer, sizeof(buffer), &sample);

		
		// Acquire latest sensor data
		//const uint64_t timestamp = sample.tick; // replace this with actual gyroscope timestamp
		FusionVector gyroscope = { sample.ang_vel[0], sample.ang_vel[1], sample.ang_vel[2] };
		
		FusionVector accelerometer = { sample.accel[0], sample.accel[1], sample.accel[2] };

		airTimestamp = sample.tick;

		//FusionVector gyroscope = { ang_vel[0], ang_vel[1], ang_vel[2] }; // replace this with actual gyroscope data in degrees/s
		//FusionVector accelerometer = { accel_vec[0], accel_vec[1], accel_vec[2] }; // replace this with actual accelerometer data in g

		// Apply calibration
		gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
		accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

		// Update gyroscope offset correction algorithm
		gyroscope = FusionOffsetUpdate(&offset, gyroscope);

		// Calculate delta time (in seconds) to account for gyroscope sample clock error
		static uint64_t previousTimestamp;
		const float deltaTime = (float)(airTimestamp - previousTimestamp) / (float)1e9;
		previousTimestamp = airTimestamp;

		// Update gyroscope AHRS algorithm
		mtx.lock();
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);
		mtx.unlock();

		//lock mutex and update values
		mtx.lock();
		qt = FusionAhrsGetQuaternion(&ahrs);
		euler = FusionQuaternionToEuler(qt);
		earth = FusionAhrsGetEarthAcceleration(&ahrs);
		mtx.unlock();

	}
	return 0;

}
int brightness = 0;
DWORD WINAPI interface4Handler(LPVOID lpParam) {
	
	//get initial brightness from device
	std::array<uint8_t, 17> initBrightness = { 0x00, 0xfd, 0x1e, 0xb9, 0xf0, 0x68, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03 };
	hid_write(device4, initBrightness.data(), initBrightness.size());
	

	while (g_isListening) {
		std::array<uint8_t, 65> recv = {};
		int res = hid_read(device4, recv.data(), recv.size());
		if (res > 0) {
			switch (recv[22]) {

			case 0x03: //Brightness down press
				it4.lock();
				brightness = recv[30];
				it4.unlock();
				break;

			case 0x02: //Brightness up press
				it4.lock();
				brightness = recv[30];
				it4.unlock();
				break;
			default:
				//std::cout << "Unknown Packet! " << (int)recv[22] << std::endl;
				break;
			}

			switch (recv[15]) {

			case 0x03: //Brightness from cmd
				it4.lock();
				brightness = recv[23];
				it4.unlock();
				break;

			default:
				//todo
				break;
			}
		}
	}
	return 0;
}


int StartConnection()
{


	if (g_isTracking) {
		std::cout << "Already Tracking" << std::endl;
		return 1;
	}
	else {
		std::cout << "Opening Device" << std::endl;
		// open device
		device = open_device();
		device4 = open_device4();
		if (!device || !device4) {
			std::cout << "Unable to open device" << std::endl;
			return 1;
		}


		std::cout << "Sending Payload" << std::endl;
		// open the floodgates
		uint8_t magic_payload[] = { 0x00, 0xaa, 0xc5, 0xd1, 0x21, 0x42, 0x04, 0x00, 0x19, 0x01 };


		int res = hid_write(device, magic_payload, sizeof(magic_payload));
		if (res < 0) {
			std::cout << "Unable to write to device" << std::endl;
			return 1;
		}
		ThreadParams trackParams = { device };
		g_isTracking = true;
		std::cout << "Tracking Starting Thread" << std::endl;


		//Start Tracking Thread
		trackThread = CreateThread(NULL, 0, track, &trackParams, 0, NULL);
		if (trackThread == NULL) {
			std::cout << "Failed to create thread" << std::endl;
			return 1;
		}

		ThreadParams listenParams = { };
		g_isListening = true;
		//Start Interface 4 listener
		listenThread = CreateThread(NULL, 0, interface4Handler, &listenParams, 0, NULL);
		if (listenThread == NULL) {
			std::cout << "Failed to create thread" << std::endl;
			return 1;
		}
		std::cout << "Listenr Thread Started" << std::endl;

		return 1;
	}
}


int StopConnection()
{
	if (g_isTracking) {
		g_isTracking = false;
		g_isListening = false;
		// Wait for the track thread to finish
		WaitForSingleObject(trackThread, INFINITE);
		TerminateThread(trackThread, 0);
		CloseHandle(trackThread);

		// Wait for the listen thread to finish
		WaitForSingleObject(listenThread, INFINITE);
		TerminateThread(listenThread, 0);
		CloseHandle(listenThread);
		return 1;
	}
	else {
		return -1;
	}
}

float* q = new float[4];

float* GetQuaternion()
{
	mtx.lock();
	q[0] = qt.array[0];
	q[1] = qt.array[1];
	q[2] = qt.array[2];
	q[3] = qt.array[3];
	mtx.unlock();
	return q;
}

float* e = new float[3];

float* GetEuler()
{

	mtx.lock();
	e[0] = euler.angle.pitch;
	e[1] = euler.angle.roll;
	e[2] = euler.angle.yaw;
	mtx.unlock();
	return e;
}


int GetBrightness()
{
	int curBrightness;
	it4.lock();
	curBrightness = brightness;
	it4.unlock();

	return curBrightness;
}

int SetFusionGain(float gain)
{
	mtx.lock();
	settings.gain = gain;
	FusionAhrsSetSettings(&ahrs, &settings);
	mtx.unlock();
	return 1;
}

int SetFusionAccelRejection(float accelReject) {
	mtx.lock();
	settings.accelerationRejection = accelReject;
	FusionAhrsSetSettings(&ahrs, &settings);
	mtx.unlock();
	return 1;
}

int SetFusionMagRejection(float magReject) {
	mtx.lock();
	settings.magneticRejection = magReject;
	FusionAhrsSetSettings(&ahrs, &settings);
	mtx.unlock();
	return 1;
}

int SetFusionRejectTimeout(unsigned int timout) {
	mtx.lock();
	settings.rejectionTimeout = timout;
	FusionAhrsSetSettings(&ahrs, &settings);
	mtx.unlock();
	return 1;
}

int GetFusionState() {
	//TODO: check if initialized/errors/ect return an int value to represent state
	// 1 finished and running
	// 0 initializing
	// -1 error
	// error counter?

	return 0;
}

float* rawGyro = new float[3];

float* GetRawGyro() {

	mtx.lock();
	rawGyro = ang_vel;
	mtx.unlock();
	return rawGyro;
}

float* rawAccel = new float[3];
float* GetRawAccel() {

	mtx.lock();
	rawAccel = accel_vec;
	mtx.unlock();
	return rawAccel;
}

float* rawMag = new float[3];
float* GetRawMag() {

	mtx.lock();
	rawMag = mag_vec;
	mtx.unlock();
	return rawMag;
}


unsigned long long GetAirTimestamp() {
	mtx.lock();
	unsigned long long ts = airTimestamp;
	mtx.unlock();
	return ts;
}

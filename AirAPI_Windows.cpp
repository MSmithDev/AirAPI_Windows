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
	uint32_t t0v, t1v, t2v, t3v;
	t0v = data[0];
	t1v = (data[1] << 8);
	t2v = (data[2] << 16);
	t3v = (data[3] << 24);

	uint32_t unsigned_value = t0v | t1v | t2v | t3v;
	return ((int32_t)unsigned_value);
}

static int32_t pack24bit_signed(const uint8_t* data) {
	uint32_t unsigned_value = (data[0]) | (data[1] << 8) | (data[2] << 16);
	if ((data[2] & 0x80) != 0) unsigned_value |= (0xFFu << 24);
	return static_cast<int32_t>(unsigned_value);
}

static int16_t pack16bit_signed(const uint8_t* data) {
	uint32_t t0v, t1v;
	t0v = data[0];
	t1v = (data[1] << 8);

	uint16_t unsigned_value = t0v | t1v;
	return static_cast<int16_t>(unsigned_value);
}

static int32_t pack32bit_signed_swap(const uint8_t* data) {
	uint32_t t0v, t1v, t2v, t3v;
	t0v = data[0] << 24;
	t1v = (data[1] << 16);
	t2v = (data[2] << 8);
	t3v = (data[3]);

	uint32_t unsigned_value = t0v | t1v | t2v | t3v;
	return static_cast<int32_t>(unsigned_value);
}


static int16_t pack16bit_signed_swap(const uint8_t* data) {
	uint32_t t0v, t1v;
	t0v = data[0] << 8;
	t1v = (data[1]);
	uint16_t unsigned_value = t0v | t1v;
	return (int16_t)unsigned_value;
}

//Global variables for get functions
static float ang_vel[3] = {};
static float accel_vec[3] = {};
static float mag_vec[3] = {};
static uint64_t airTimestamp;

static int parse_report(AirDataPacket* packet, int size, air_sample* out_sample) {


	const uint64_t timestamp = packet->timestamp;

	out_sample->tick = timestamp;
	
	//Gyro scaling values
	int32_t vel_m = pack16bit_signed(packet->angular_multiplier);
	int32_t vel_d = pack32bit_signed(packet->angular_divisor);
	
	//Gyro conversion to 32bit signed
	int32_t vel_x = pack24bit_signed(packet->angular_velocity_x);
	int32_t vel_y = pack24bit_signed(packet->angular_velocity_y);
	int32_t vel_z = pack24bit_signed(packet->angular_velocity_z);

	//Gyro scale correction
	float vel_x_cor = (float)vel_z * (float)vel_m / (float)vel_d;
	float vel_y_cor = (float)vel_x * (float)vel_m / (float)vel_d;
	float vel_z_cor = (float)vel_y * (float)vel_m / (float)vel_d;

	//Fusion gyro sample
	out_sample->ang_vel[0] = vel_x_cor;
	out_sample->ang_vel[1] = vel_y_cor;
	out_sample->ang_vel[2] = vel_z_cor;

	//Accel scaling values
	int32_t accel_m = pack16bit_signed(packet->acceleration_multiplier);
	int32_t accel_d = pack32bit_signed(packet->acceleration_divisor);

	//Accel conversion to 32bit signed int
	int32_t accel_x = pack24bit_signed(packet->acceleration_x);
	int32_t accel_y = pack24bit_signed(packet->acceleration_y);
	int32_t accel_z = pack24bit_signed(packet->acceleration_z);


	float accel_x_cor = (float)accel_x * (float)accel_m / (float)accel_d;
	float accel_y_cor = (float)accel_y * (float)accel_m / (float)accel_d;
	float accel_z_cor = (float)accel_z * (float)accel_m / (float)accel_d;

	//Fusion Accel Sample
	out_sample->accel[0] = accel_x_cor;
	out_sample->accel[1] = accel_y_cor;
	out_sample->accel[2] = accel_z_cor;

	int32_t magnet_m = pack16bit_signed_swap(packet->magnetic_multiplier);
	int32_t magnet_d = pack32bit_signed_swap(packet->magnetic_divisor);

	int16_t magnet_x = pack16bit_signed_swap(packet->magnetic_x);
	int16_t magnet_y = pack16bit_signed_swap(packet->magnetic_y);
	int16_t magnet_z = pack16bit_signed_swap(packet->magnetic_z);

	float mag_x_cor = (float)magnet_x * (float)magnet_m / (float)magnet_d;
	float mag_y_cor = (float)magnet_y * (float)magnet_m / (float)magnet_d;
	float mag_z_cor = (float)magnet_z * (float)magnet_m / (float)magnet_d;

	//Fusion Mag Sample
	out_sample->mag[0] = mag_x_cor;
	out_sample->mag[1] = mag_y_cor;
	out_sample->mag[2] = mag_z_cor;


	//lock and update the global variables
	mtx.lock();
	
	airTimestamp = timestamp;

	ang_vel[0] = vel_x_cor;
	ang_vel[1] = vel_y_cor;
	ang_vel[2] = vel_z_cor;

	accel_vec[0] = accel_x_cor;
	accel_vec[1] = accel_y_cor;
	accel_vec[2] = accel_z_cor;

	mag_vec[0] = mag_x_cor;
	mag_vec[1] = mag_y_cor;
	mag_vec[2] = mag_z_cor;
	mtx.unlock();


	return 1;
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


uint64_t GetAirTimestamp() {
	mtx.lock();
	uint64_t ts = airTimestamp;
	mtx.unlock();
	return ts;
}

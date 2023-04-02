// TODO: post-draft pr, remove bg / investigation comments and links

// macOS-specific includes
#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__)) // http://web.archive.org/web/20191012035921/http://nadeausoftware.com/articles/2012/01/c_c_tip_how_use_compiler_predefined_macros_detect_operating_system#BSD

#include "mac.h"

// absolute path for installation via homebrew
// to install: `brew install hidapi`
#include "/usr/local/Cellar/hidapi/0.13.1/include/hidapi/hidapi.h"
#include "deps/Fusion/Fusion/build/include/Fusion.h"

#else

// windows-specific includes
#include "pch.h"
#include "deps/hidapi-win/include/hidapi.h"
#include "deps/Fusion/Fusion/Fusion.h"

#endif

// mac & windows adapted
#include "AirAPI_Windows.h"

// mac & windows compatible
#include <assert.h>
#include <pthread.h>
#include <iostream>
#include <string>
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

pthread_t trackThread;
pthread_t listenThread;

hid_device* device;
hid_device* device4;

#define SAMPLE_RATE (1000) // replace this with actual sample rate

// task thread mutexs
pthread_mutex_t mtx;
pthread_mutex_t it4;

// connection close config
bool signalled;
pthread_mutex_t mutex;
pthread_cond_t cond;

typedef struct {
	uint64_t tick;
	int32_t ang_vel[3];
	int32_t accel[3];
} air_sample;

static int
parse_report(const unsigned char* buffer_in, int size, air_sample* out_sample)
{
	if (size != 64) {
		printf("Invalid packet size");
		return -1;
	}
	// clock in nanoseconds
	buffer_in += 4;
	out_sample->tick = ((uint64_t) * (buffer_in++));
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 8);
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 16);
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 24);
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 32);
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 40);
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 48);
	out_sample->tick = out_sample->tick | (((uint64_t) * (buffer_in++)) << 56);

	uint32_t t0v, t1v, t2v, t3v, t0a, t1a, t2a, t3a;
	// gyroscope measurements	
	buffer_in += 6;
	if (*(buffer_in + 2) & 0x80) {
		t0v = (0xff << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample->ang_vel[0] = t0v | t1v | t2v | t3v;
		// out_sample->ang_vel[0] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else {
		t0v = (0x00 << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample->ang_vel[0] = t0v | t1v | t2v | t3v;
		// out_sample->ang_vel[0] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	if (*(buffer_in + 2) & 0x80) {
		t0v = (0xff << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample->ang_vel[1] = t0v | t1v | t2v | t3v;
		// out_sample->ang_vel[1] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else {
		t0v = (0x00 << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample->ang_vel[1] = t0v | t1v | t2v | t3v;
		// out_sample->ang_vel[1] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	if (*(buffer_in + 2) & 0x80) {
		t0v = (0xff << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample->ang_vel[2] = t0v | t1v | t2v | t3v;
		// out_sample->ang_vel[2] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else {
		t0v = (0x00 << 24);
		t3v = *(buffer_in++);
		t1v = (*(buffer_in++) << 8);
		t2v = (*(buffer_in++) << 16);

		out_sample->ang_vel[2] = t0v | t1v | t2v | t3v;
		// out_sample->ang_vel[2] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	// accelerometer data
	buffer_in += 6;
	if (*(buffer_in + 2) & 0x80) {
		t0a = (0xff << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample->accel[0] = t0a | t1a | t2a | t3a;
		// out_sample->accel[0] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else {
		t0a = (0x00 << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample->accel[0] = t0a | t1a | t2a | t3a;
		// out_sample->accel[0] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	if (*(buffer_in + 2) & 0x80) {
		t0a = (0xff << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample->accel[1] = t0a | t1a | t2a | t3a;
		// out_sample->accel[1] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else {
		t0a = (0x00 << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample->accel[1] = t0a | t1a | t2a | t3a;
		// out_sample->accel[1] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	if (*(buffer_in + 2) & 0x80) {
		t0a = (0xff << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample->accel[2] = t0a | t1a | t2a | t3a;
		// out_sample->accel[2] = (0xff << 24) | *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}
	else {
		t0a = (0x00 << 24);
		t3a = *(buffer_in++);
		t1a = (*(buffer_in++) << 8);
		t2a = (*(buffer_in++) << 16);

		out_sample->accel[2] = t0a | t1a | t2a | t3a;
		// out_sample->accel[2] = *(buffer_in++) | (*(buffer_in++) << 8) | (*(buffer_in++) << 16);
	}

	// std::cout << out_sample << std::endl;

	return 0;
}


static void
process_ang_vel(const int32_t in_ang_vel[3], float out_vec[])
{

	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_ang_vel[0]) * -1.0f * GYRO_SCALAR;
	out_vec[1] = (float)(in_ang_vel[2]) * GYRO_SCALAR;
	out_vec[2] = (float)(in_ang_vel[1]) * GYRO_SCALAR;
}

static void
process_accel(const int32_t in_accel[3], float out_vec[])
{
	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_accel[0]) * ACCEL_SCALAR;
	out_vec[1] = (float)(in_accel[2]) * ACCEL_SCALAR;
	out_vec[2] = (float)(in_accel[1]) * ACCEL_SCALAR;

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

// must be of format:
// const pthread_attr_t *attr

struct ThreadParams {
	hid_device* device;
};

// must conform like example:
// void *worker_thread(void *arg)
void *track(void *lpParam) {

	//Thread to handle tracking
	unsigned char buffer[64] = {};
	uint64_t last_sample_tick = 0;
	air_sample sample = {};
	ThreadParams* params = static_cast<ThreadParams*>(lpParam);
	//static FusionVector ang_vel = {}, accel_vec = {};
	static float ang_vel[3] = {};
	static float accel_vec[3] = {};


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
	FusionAhrs ahrs;

	FusionOffsetInitialise(&offset, SAMPLE_RATE);
	FusionAhrsInitialise(&ahrs);


	// Set AHRS algorithm settings
	const FusionAhrsSettings settings = {
			.gain = 0.5f,
			.accelerationRejection = 10.0f,
			.magneticRejection = 20.0f,
			.rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
	};
	FusionAhrsSetSettings(&ahrs, &settings);

	while (g_isTracking) {
		try {
			// code that might throw an exception
			int res = hid_read(device, buffer, sizeof(buffer));
			if (res < 0) {
				break;
			}
		}
		catch (const std::exception& e) {
			signalled = true;
			// handle the exception
			std::cerr << e.what();
		}


		//parse
		parse_report(buffer, sizeof(buffer), &sample);

		//process sample
		process_ang_vel(sample.ang_vel, ang_vel);
		process_accel(sample.accel, accel_vec);

		// Acquire latest sensor data
		const uint64_t timestamp = sample.tick; // replace this with actual gyroscope timestamp
		FusionVector gyroscope = { ang_vel[0], ang_vel[1], ang_vel[2] }; // replace this with actual gyroscope data in degrees/s
		FusionVector accelerometer = { accel_vec[0], accel_vec[1], accel_vec[2] }; // replace this with actual accelerometer data in g

		// Apply calibration
		gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
		accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);

		// Update gyroscope offset correction algorithm
		gyroscope = FusionOffsetUpdate(&offset, gyroscope);

		// Calculate delta time (in seconds) to account for gyroscope sample clock error
		static uint64_t previousTimestamp;
		const float deltaTime = (float)(timestamp - previousTimestamp) / (float)1e9;
		previousTimestamp = timestamp;

		// Update gyroscope AHRS algorithm
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

		//lock mutex and update values
		pthread_mutex_lock(&mtx);
		qt = FusionAhrsGetQuaternion(&ahrs);
		euler = FusionQuaternionToEuler(qt);
		earth = FusionAhrsGetEarthAcceleration(&ahrs);
		pthread_mutex_unlock(&mtx);
		// signal
		signalled = true;
	}
	return 0;
}

// needs to conform to void *_Nullable (*_Nonnull)(void *)
// LPVOID is alias of: void *
// aka pointer to any type
//
// DWORD is typealias of: unsigned int
// 
// must be format: void *worker_thread(void *arg)
void *interface4Handler(void *lpParam) {
	std::cout << "interface4Handler invoked" << std::endl;

	//get initial brightness from device
	std::array<uint8_t, 17> initBrightness = { 0x00, 0xfd, 0x1e, 0xb9, 0xf0, 0x68, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03 };
	hid_write(device4, initBrightness.data(), initBrightness.size());	

	while (g_isListening) {
		std::array<uint8_t, 65> recv = {};
		int res = hid_read(device4, recv.data(), recv.size());
		if (res > 0) {
			switch (recv[22]) {

			case 0x03: //Brightness down press
				pthread_mutex_lock(&it4);
				brightness = recv[30];
				pthread_mutex_unlock(&it4);
				
				std::cout << "brightness: down detected, value:" << (int)brightness << std::endl;
				// signal
				signalled = true;
				break;

			case 0x02: //Brightness up press
				pthread_mutex_lock(&it4);
				brightness = recv[30];
				pthread_mutex_unlock(&it4);

				std::cout << "brightness: up detected, value:" << (int)brightness << std::endl;
				// signal
				signalled = true;
				break;

			default:
				// std::cout << "Brightness: Unknown Packet! " << (int)recv[22] << std::endl;
				break;

			}

			switch (recv[15]) {

			case 0x03: //Brightness from cmd
				pthread_mutex_lock(&it4);
				brightness = recv[23];
				pthread_mutex_unlock(&it4);

				std::cout << "Brightness: from cmd" << std::endl;
				// signal
				signalled = true;
				break;

			default:
				std::cout << "Brightness: default inner switch case" << std::endl;
				break;
			}
		} else {
			std::cout << "Brightness: connection failed due to non-response" << std::endl;
		}
	}

	signalled = true;
}
int StartConnection()
{
	if (g_isTracking) {
		std::cout << "Already Tracking" << std::endl;
		return 1;
	}
	else {
		//
		// ** Init Device Connection **//
		//
		std::cout << "Opening Device" << std::endl;
		// open device
		device = open_device();
		device4 = open_device4();
		if (!device || !device4) {
			std::cout << "Unable to open device" << std::endl;
			return 1;
		}

		//
		// ** Start Device Communication **//
		//
		std::cout << "Sending Payload" << std::endl;
		// open the floodgates
		uint8_t magic_payload[] = { 0x00, 0xaa, 0xc5, 0xd1, 0x21, 0x42, 0x04, 0x00, 0x19, 0x01 };
		const size_t payloadArrSize = sizeof(magic_payload) / sizeof(magic_payload[0]);

		std::cout << "magic payload: ";
		for (size_t i = 0; i < payloadArrSize; i++) {
			std::cout << static_cast<int>(magic_payload[i]) << " ";
		}
		std::cout << std::endl;

		int res = hid_write(device, magic_payload, sizeof(magic_payload));
		if (res < 0) {
			std::cout << "Unable to write to device" << std::endl;
			return 1;
		}
		std::cout << res << std::endl;
		
		//
		// ** Tracking Thread Creation **//
		//
		ThreadParams *trackParamsPtr;
		trackParamsPtr = (ThreadParams *) malloc(sizeof(ThreadParams));
		trackParamsPtr->device = { device };
		g_isTracking = true;
		std::cout << "Starting tracking thread" << std::endl;
		// thread creation
		pthread_create(&trackThread, NULL, &track, trackParamsPtr);
		
		if (trackThread == NULL) {
			std::cout << "Failed to start tracking thread" << std::endl;
			return 1;
		}
		std::cout << "Starting tracking thread" << std::endl;

		//
		// ** Listening Thread Creation **//
		//
		ThreadParams *listenParamsPtr;
		listenParamsPtr = (ThreadParams *) malloc(sizeof(ThreadParams));
		listenParamsPtr->device = { };
		g_isListening = true;
		std::cout << "Starting listening thread" << std::endl;
		// thread creation
		pthread_create(&listenThread, NULL, &interface4Handler, listenParamsPtr);
		if (listenThread == NULL) {
			std::cout << "Failed to create listening thread" << std::endl;
			return 1;
		}
		std::cout << "Listening thread started" << std::endl;
		return 1;
	}
}

int StopConnection()
{
	if (g_isTracking) {
		g_isTracking = false;
		g_isListening = false;
		// need to convert to GCC compatible for macOS
		// Wait for the track thread to finish
		std::cout << "stopping connection" << std::endl;

		// init condition & mutex
        pthread_mutex_init(&mutex, NULL);
        pthread_cond_init(&cond, NULL);
		
		// wait to exit tracking thread
		pthread_mutex_lock(&mutex);
		while (!signalled) {
			pthread_mutex_lock(&mutex);
			pthread_cond_wait(&cond, &mtx);
			pthread_mutex_unlock(&mutex);
		} signalled = false;
		std::cout << "tracking thread has exited" << std::endl;

		// exit tracking thread
		pthread_exit(&trackThread);

		// wait to exit tracking thread
		while (!signalled) {
			pthread_mutex_lock(&mutex);
			pthread_cond_wait(&cond, &mtx);
			pthread_mutex_unlock(&mutex);
		} signalled = false;
		std::cout << "listening thread has exited" << std::endl;
		
		// exit tracking thread
		pthread_exit(&listenThread);
		
		return 1;
	}
	else {
		return -1;
	}
}

float* q = new float[4];

float* GetQuaternion()
{
	pthread_mutex_lock(&mtx);
	q[0] = qt.array[0];
	q[1] = qt.array[1];
	q[2] = qt.array[2];
	q[3] = qt.array[3];
	pthread_mutex_unlock(&mtx);

	// signal
	signalled = true;
	return q;
}

float* e = new float[3];

float* GetEuler()
{

	pthread_mutex_lock(&mtx);
	e[0] = euler.angle.pitch;
	e[1] = euler.angle.roll;
	e[2] = euler.angle.yaw;
	pthread_mutex_unlock(&mtx);
	// signal
	signalled = true;
	return e;
}

int brightness = 0;

int GetBrightness()
{
	int curBrightness;
	pthread_mutex_lock(&mtx);
	curBrightness = brightness;
	pthread_mutex_unlock(&mtx);
	// signal
	signalled = true;
	return curBrightness;
}

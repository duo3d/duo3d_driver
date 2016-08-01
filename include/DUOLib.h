//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// This library is part of DUO SDK that allows the use of DUO devices in your own applications
//
// For updates and file downloads go to: http://duo3d.com/
//
// Copyright 2014-2016 (c) Code Laboratories, Inc.  All rights reserved.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifndef _DUOLIB_H
#define _DUOLIB_H

#include <stdint.h>

#ifdef WIN32
    #ifdef DUOLIB_DLL
		#define API_FUNCTION(type)	__declspec(dllexport) type __cdecl
	#else
		#define API_FUNCTION(type)	__declspec(dllimport) type __cdecl
	#endif
    #define CALLBACK    			__stdcall
#else
	#define API_FUNCTION(type)	 	__attribute__((visibility("default"))) type
	#define CALLBACK
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DUO C API
extern "C" {

// DUO instance
typedef void *DUOInstance;

// DUO binning
enum DUOBinning
{
	DUO_BIN_ANY = -1,
	DUO_BIN_NONE = 0,
	DUO_BIN_HORIZONTAL2 = 1,        // Horizontal binning by factor of 2
	DUO_BIN_HORIZONTAL4 = 2,        // Horizontal binning by factor of 4
	DUO_BIN_VERTICAL2 = 4,          // Vertical binning by factor of 2
	DUO_BIN_VERTICAL4 = 8           // Vertical binning by factor of 4
};

// DUO resolution info
typedef struct
{
    int   width;                    // DUO frame width
    int   height;                   // DUO frame height
    int   binning;                  // DUO image binning
    float fps;                      // DUO frame rate
    float minFps;                   // DUO minimum frame rate for selected resolution and binning
    float maxFps;                   // DUO maximum frame rate for selected resolution and binning
}DUOResolutionInfo, *PDUOResolutionInfo;

// DUO IMU data sample
typedef struct
{
    uint32_t timeStamp;             // DUO IMU time stamp in 100us increments
    float tempData;                 // DUO temperature data in degrees Centigrade
    float accelData[3];             // DUO accelerometer data (x,y,z) in g units
    float gyroData[3];      		// DUO gyroscope data (x,y,z) id degrees/s
}DUOIMUSample;

#define DUO_MAX_IMU_SAMPLES     100

// DUO Frame
typedef struct
{
    uint32_t     width;				// DUO frame width
    uint32_t     height;			// DUO frame height
    uint8_t      ledSeqTag;			// DUO frame LED tag
    uint32_t     timeStamp;			// DUO frame time stamp in 100us increments
    uint8_t     *leftData;			// DUO left frame data
    uint8_t     *rightData;			// DUO right frame data
    uint8_t      IMUPresent;        // True if IMU chip is present (DUO MLX)
    uint32_t     IMUSamples;        // Number of IMU data samples in this frame
    DUOIMUSample IMUData[DUO_MAX_IMU_SAMPLES];	// DUO IMU data samples
}DUOFrame, *PDUOFrame;

// DUO LED PWM
typedef struct  
{
	uint8_t ledPwmValue[4];			// LED PWM values are in percentage [0,100]
}DUOLEDSeq, *PDUOLEDSeq;

// DUO Accelerometer Range
enum DUOAccelRange
{
    DUO_ACCEL_2G = 0,				// DUO Accelerometer full scale range +/- 2g (default)
    DUO_ACCEL_4G,               	// DUO Accelerometer full scale range +/- 4g
    DUO_ACCEL_8G,               	// DUO Accelerometer full scale range +/- 8g
    DUO_ACCEL_16G               	// DUO Accelerometer full scale range +/- 16g
};

// DUO Gyroscope Range
enum DUOGyroRange
{
    DUO_GYRO_250 = 0,				// DUO Gyroscope full scale range 250 deg/s (default)
    DUO_GYRO_500,               	// DUO Gyroscope full scale range 500 deg/s
    DUO_GYRO_1000,               	// DUO Gyroscope full scale range 1000 deg/s
    DUO_GYRO_2000               	// DUO Gyroscope full scale range 2000 deg/s
};

#pragma pack(push, 1)
struct DUO_INTR
{
    uint16_t width;
    uint16_t height;
    struct INTR
    {
        double k1, k2, k3;          // Camera radial distortion coefficients
        double k4, k5, k6;          // Camera radial distortion coefficients
        double p1, p2;              // Camera tangential distortion coefficients
        double fx, fy;              // Camera focal lengths in pixel units
        double cx, cy;              // Camera principal point
    };
    INTR left;
    INTR right;
};
struct DUO_EXTR
{
    double rotation[9];
    double translation[3];
};
struct DUO_STEREO
{
    double M1[9], M2[9];            // 3x3 - Camera matrices
    double D1[8], D2[8];            // 1x8 - Camera distortion parameters
    double R[9];                    // 3x3 - Rotation between left and right camera
    double T[3];                    // 3x1 - Translation vector between left and right camera
    double R1[9], R2[9];            // 3x3 - Rectified rotation matrices
    double P1[12], P2[12];          // 3x4 - Rectified projection matrices
    double Q[16];                   // 4x4 - Disparity to depth mapping matrix
};
#pragma pack(pop)

// Get DUOLib version string
API_FUNCTION(char*) GetDUOLibVersion();

// DUO resolution enumeration
// To enumerate resolution settings for specific resolution, set width and height and optionally fps.
// To enumerate all supported resolutions set width, height and fps all to -1.
// NOTE: There are large number of resolution settings supported (7824 total)
API_FUNCTION(int) EnumerateDUOResolutions(DUOResolutionInfo *resList, int32_t resListSize,
                                          int32_t width = -1, int32_t height = -1,
                                          int32_t binning = DUO_BIN_ANY, float fps = -1);

// DUO device initialization
API_FUNCTION(bool) OpenDUO(DUOInstance *duo);
API_FUNCTION(bool) CloseDUO(DUOInstance duo);

// DUO frame callback function
// NOTE: This function is called in the context of the DUO capture thread.
//		 To prevent any dropped frames, this function must return as soon as possible.
typedef void (CALLBACK *DUOFrameCallback)(const PDUOFrame pFrameData, void *pUserData);

// DUO device capture control
API_FUNCTION(bool) StartDUO(DUOInstance duo, DUOFrameCallback frameCallback, void *pUserData, bool masterMode = true);
API_FUNCTION(bool) StopDUO(DUOInstance duo);

// Get DUO parameters
// Returns DUO device name in user allocateed string (min size 252 bytes)
API_FUNCTION(bool) GetDUODeviceName(DUOInstance duo, char *val);
// Returns DUO serial number in user allocateed string (min size 252 bytes)
API_FUNCTION(bool) GetDUOSerialNumber(DUOInstance duo, char *val);
// Returns DUO firmware version in user allocateed string (min size 252 bytes)
API_FUNCTION(bool) GetDUOFirmwareVersion(DUOInstance duo, char *val);
// Returns DUO firmware build information in user allocateed string (min size 252 bytes)
API_FUNCTION(bool) GetDUOFirmwareBuild(DUOInstance duo, char *val);
// Returns currently selected DUO resolution info
API_FUNCTION(bool) GetDUOResolutionInfo(DUOInstance duo, DUOResolutionInfo *resInfo);
// Returns DUO frame width and height
API_FUNCTION(bool) GetDUOFrameDimension(DUOInstance duo, uint32_t *width, uint32_t *height);
// Returns DUO exposure value in percentage [0,100]
API_FUNCTION(bool) GetDUOExposure(DUOInstance duo, double *val);
// Returns DUO exposure value in milliseconds
API_FUNCTION(bool) GetDUOExposureMS(DUOInstance duo, double *val);
// Returns DUO auto exposure value
API_FUNCTION(bool) GetDUOAutoExposure(DUOInstance duo, bool *val);
// Returns DUO gain value in percentage [0,100]
API_FUNCTION(bool) GetDUOGain(DUOInstance duo, double *val);
// Returns DUO horizontal image flip value
API_FUNCTION(bool) GetDUOHFlip(DUOInstance duo, bool *val);
// Returns DUO vertical image flip value
API_FUNCTION(bool) GetDUOVFlip(DUOInstance duo, bool *val);
// Returns DUO camera swap value
API_FUNCTION(bool) GetDUOCameraSwap(DUOInstance duo, bool *val);
// Returns DUO LED brightness in percentage [0,100]
API_FUNCTION(bool) GetDUOLedPWM(DUOInstance duo, double *val);
// Returns DUO calibration present status value
API_FUNCTION(bool) GetDUOCalibrationPresent(DUOInstance duo, bool *val);
// Returns DUO field of view for currently selected resolution. User must allocate 4 double values: (leftHFOV, leftVFOV, rightHFOV, rightVFOV)
API_FUNCTION(bool) GetDUOFOV(DUOInstance duo, double *val);
// Returns DUO rectified field of view for currently selected resolution. User must allocate 4 double values: (leftHFOV, leftVFOV, rightHFOV, rightVFOV)
API_FUNCTION(bool) GetDUORectifiedFOV(DUOInstance duo, double *val);
// Returns DUO image undistort value
API_FUNCTION(bool) GetDUOUndistort(DUOInstance duo, bool *val);
// Returns DUO camera intrinsics parameters, see DUO_INTR structure
API_FUNCTION(bool) GetDUOIntrinsics(DUOInstance duo, DUO_INTR *val);
// Returns DUO camera extrinsics parameters, see DUO_EXTR structure
API_FUNCTION(bool) GetDUOExtrinsics(DUOInstance duo, DUO_EXTR *val);
// Returns DUO camera stereo parameters, see DUO_STEREO structure
API_FUNCTION(bool) GetDUOStereoParameters(DUOInstance duo, DUO_STEREO *val);
// Returns DUO currently selected IMU range
API_FUNCTION(bool) GetDUOIMURange(DUOInstance duo, int *accel, int *gyro);

// Set DUO parameters
// Set current resolution for DUO.
// The DUOResolutionInfo is obtained by calling EnumerateDUOResolutions with desired image size, binning and frame rate.
API_FUNCTION(bool) SetDUOResolutionInfo(DUOInstance duo, DUOResolutionInfo resInfo);
// Sets DUO exposure value in percentage [0,100]
API_FUNCTION(bool) SetDUOExposure(DUOInstance duo, double val);
// Sets DUO exposure value in milliseconds
API_FUNCTION(bool) SetDUOExposureMS(DUOInstance duo, double val);
// Sets DUO auto exposure value, default: false. The target exposure value is set using SetDUOExposure.
API_FUNCTION(bool) SetDUOAutoExposure(DUOInstance duo, bool val);
// Sets DUO gain value in percentage [0,100], default: 0
API_FUNCTION(bool) SetDUOGain(DUOInstance duo, double val);
// Sets DUO horizontal image flip value, default: false
API_FUNCTION(bool) SetDUOHFlip(DUOInstance duo, bool val);
// Sets DUO vertical image flip value, default: false
API_FUNCTION(bool) SetDUOVFlip(DUOInstance duo, bool val);
// Sets DUO camera swap value, default: false
API_FUNCTION(bool) SetDUOCameraSwap(DUOInstance duo, bool val);
// Sets DUO LED brightness in percentage [0,100], default: 0
API_FUNCTION(bool) SetDUOLedPWM(DUOInstance duo, double val);
// Sets DUO LED sequence, see DUOLEDSeq, default: none
API_FUNCTION(bool) SetDUOLedPWMSeq(DUOInstance duo, PDUOLEDSeq val, uint32_t size);
// Sets DUO image undistort value, default: false
API_FUNCTION(bool) SetDUOUndistort(DUOInstance duo, bool val);
// Sets DUO IMU range, default: DUO_ACCEL_2G, DUO_GYRO_250
API_FUNCTION(bool) SetDUOIMURange(DUOInstance duo, int accel, int gyro);
// Sets DUO IMU sampling rate [50,500] Hz, default: 100Hz
API_FUNCTION(bool) SetDUOIMURate(DUOInstance duo, double rate);

} // extern "C"

#endif // _DUOLIB_H

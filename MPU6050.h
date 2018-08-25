#pragma once
#include <Wire.h>
#include<math.h>

#define CALIBRATION_ITERATION 2000
#define SPATIAL_DIMENSIONS 3
#define RPY_MIX_PROPORTIONS 0.95
#define MPU_PI 3.1415926
#define RAD2DEG 180. / MPU_PI
#define DEG2RAD MPU_PI / 180.
class RPY
{
public:
	double pitch;
	double yaw;
	double roll;

	RPY()
	{
		pitch = 0;
		yaw = 0;
		roll = 0;
	}
  
};


class MPU6050
{
private:
	double gyro_rawAngularSpeed[SPATIAL_DIMENSIONS] = { 0 };
	double accel_Raw[SPATIAL_DIMENSIONS] = { 0 };

	double gyro_Calibration[SPATIAL_DIMENSIONS] = { 0 };
	double accel_Calibration[SPATIAL_DIMENSIONS] = { 0 };

public:

	double gyro_angularPosition[SPATIAL_DIMENSIONS] = { 0 };
	double accel_Total[SPATIAL_DIMENSIONS] = { 0 };


	RPY gyro_RPY;
	RPY accel_RPY;


	void UpdateMPU(double timeSinceLastUpdate);

	void CalcAccelRPY();
	void CalibrateMPU();
	
	void RecordGyroRegisters();
	void ProcessGyroData();

	void RecordAccelRegisters();
	void ProcessAccelData();

	void ConnectMPU();


	MPU6050();
	~MPU6050();

};




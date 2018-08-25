#include "MPU6050.h"

double clamp(double val)
{
	if (val > 1)
	{
		val = 0.999999;
	}
	else if (val <-1)
	{
		val = -0.999999;
	}
	return val;
}
void MPU6050::CalcAccelRPY()
{

	double total = 0;
	for (int i = 0; i < SPATIAL_DIMENSIONS; i++)
	{total += accel_Total[i] * accel_Total[i];}
		
	double acc_totalVector = sqrt(total);
	
	accel_RPY.roll = asin(clamp(accel_Total[1] / acc_totalVector)) * RAD2DEG;
	accel_RPY.pitch = -asin(clamp(accel_Total[0] / acc_totalVector)) * RAD2DEG;
}

void MPU6050::RecordAccelRegisters() 
{	
	Wire.beginTransmission(0b1101000); //I2C address of the MPU
	Wire.write(0x3B); //Starting register for Accel Readings
	Wire.endTransmission();
	Wire.requestFrom(0b1101000, 6); //Request Accel Registers (3B - 40)limit switchlimit switch
	while (Wire.available() < 6);
	accel_Raw[0] = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
	accel_Raw[1] = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
	accel_Raw[2] = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
	ProcessAccelData();
}	

void MPU6050::ProcessAccelData()
{
	for (int index = 0; index < SPATIAL_DIMENSIONS; ++index)
	{
		accel_Total[index] = (accel_Raw[index]) / 16384.0 - accel_Calibration[index];
	}

}

void MPU6050::RecordGyroRegisters()
{ 
	Wire.beginTransmission(0b1101000); //I2C address of the MPU
	Wire.write(0x43); //Starting register for Gyro Readings
	Wire.endTransmission();
	Wire.requestFrom(0b1101000, 6); //Request Gyro Registers (43 - 48)
	while (Wire.available() < 6);
	gyro_rawAngularSpeed[0] = Wire.read() << 8 | Wire.read(); //Store first two bytes into accelX
	gyro_rawAngularSpeed[1] = Wire.read() << 8 | Wire.read(); //Store middle two bytes into accelY
	gyro_rawAngularSpeed[2] = Wire.read() << 8 | Wire.read(); //Store last two bytes into accelZ
} 

void MPU6050::ProcessGyroData()
{ 
	for (int index = 0; index < SPATIAL_DIMENSIONS; index++)
	{
		gyro_rawAngularSpeed[index] -= gyro_Calibration[index];
	}
  
	gyro_rawAngularSpeed[0] = gyro_rawAngularSpeed[0] / 131.0; // 131 LSB/(deg/s)
	gyro_rawAngularSpeed[1] = gyro_rawAngularSpeed[1] / 131.0;
	gyro_rawAngularSpeed[2] = gyro_rawAngularSpeed[2] / 131.0;
}	


void MPU6050::CalibrateMPU()
{
	for (int averageItr = 0 ; averageItr < CALIBRATION_ITERATION ; ++averageItr)
	{	
		RecordGyroRegisters();
		for (int index = 0 ; index < SPATIAL_DIMENSIONS ; index++)
		{gyro_Calibration[index] += gyro_rawAngularSpeed[index];}
	}	

	for (int index = 0 ; index < SPATIAL_DIMENSIONS ; index++)
	{	
		gyro_Calibration[index] /= (double)CALIBRATION_ITERATION;
		accel_Calibration[index] = 0;
	}
	
	RecordAccelRegisters();

	double total = 0;
	for (int index = 0; index < SPATIAL_DIMENSIONS; index++)
	{	
		total += accel_Total[index] * accel_Total[index];
	}	
	total = sqrt(total);

	for (int index = 0; index < SPATIAL_DIMENSIONS; index++)
	{accel_Calibration[index] = accel_Total[index] - accel_Total[index] / total;}
	
	CalcAccelRPY();

	gyro_angularPosition[0] = accel_RPY.roll;
	gyro_angularPosition[1] = accel_RPY.pitch;
}	

void MPU6050::UpdateMPU(double timeSinceLastUpdate)
{
	RecordGyroRegisters();
	ProcessGyroData();
	
	for (int index = 0; index < 3; index++)
	{
		double val = (gyro_rawAngularSpeed[index]) * timeSinceLastUpdate;
		gyro_angularPosition[index] += val;
	}

	double sinValHolder = sin(gyro_rawAngularSpeed[2] * timeSinceLastUpdate * DEG2RAD);
	gyro_angularPosition[0] += gyro_angularPosition[1] * sinValHolder;
	gyro_angularPosition[1] -= gyro_angularPosition[0] * sinValHolder;

	RecordAccelRegisters();
	CalcAccelRPY();

	gyro_angularPosition[0] = gyro_angularPosition[0] * (double)RPY_MIX_PROPORTIONS + accel_RPY.roll * (1. - (double)RPY_MIX_PROPORTIONS);
	gyro_angularPosition[1] = gyro_angularPosition[1] * (double)RPY_MIX_PROPORTIONS + accel_RPY.pitch * (1. - (double)RPY_MIX_PROPORTIONS);

	gyro_RPY.roll = gyro_angularPosition[0];
	gyro_RPY.pitch = gyro_angularPosition[1];


}
void MPU6050::ConnectMPU()
{
	Wire.begin();
	Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
	Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
	Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
	Wire.endTransmission();
	Wire.beginTransmission(0b1101000); //I2C address of the MPU
	Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
	Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s
	Wire.endTransmission();
	Wire.beginTransmission(0b1101000); //I2C address of the MPU
	Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
	Wire.write(0b00000000); //Setting the accel to +/- 2g
	Wire.endTransmission();
}
MPU6050::MPU6050()
{}

MPU6050::~MPU6050()
{}


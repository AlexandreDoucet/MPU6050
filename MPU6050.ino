
#include <Wire.h>
#include "MPU6050.h"

#define LED_PIN 13
#define PRESCALER 8
#define CMP 999


unsigned long deltaTime = 0;
double interuptPeriod = 0;

MPU6050 mpu;

bool ledState = false;
bool printing = false;
bool interruptPrinting = false;

void setup()
{
  printing = false;
  interruptPrinting = false;
  interuptPeriod = 1. / (16000000. / ((double)PRESCALER * ((double)CMP + 1.)));

  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);

  mpu.ConnectMPU();
  mpu.CalibrateMPU();
  SetUpTimerInterrupt();
  
} 

void loop()
{
	mpu.UpdateMPU(deltaTime * interuptPeriod);
	deltaTime = 0;
	PrintToSerial(mpu);
}

void PrintToSerial(MPU6050& _mpu)
{
  if (interruptPrinting && !printing)
  {
    printing = true;
	Serial.print(_mpu.gyro_RPY.pitch, 4);
	Serial.print("\t");
	Serial.print(_mpu.gyro_RPY.roll, 4);
	Serial.print("\t");
	Serial.print(_mpu.gyro_RPY.yaw, 4);
	Serial.print("\t");

	Serial.print(_mpu.accel_RPY.pitch, 4);
	Serial.print("\t");
	Serial.print(_mpu.accel_RPY.roll, 4);
	Serial.print("\t");
  
	Serial.print(0);
	Serial.print("\t");
  Serial.print(90);
  Serial.print("\t");
  Serial.print(-90);
  Serial.print("\t");

    Serial.println();
    printing = false;
  }

}

void SetUpTimerInterrupt()
{
	cli();//stop interrupts

		  //set timer1 interrupt at 1Hz
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1 = 0;//initialize counter value to 0
			  // set compare match register for 1hz increments
	OCR1A = CMP;///15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
				// turn on CTC mode
	TCCR1B |= (1 << WGM12);

	TCCR1B |= (1 << CS01);
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);

	sei();//allow interrupts


}

ISR(TIMER1_COMPA_vect)
{
	deltaTime += 1;
	interruptPrinting = true;
	ledState = !ledState;
	digitalWrite(13, ledState);
}

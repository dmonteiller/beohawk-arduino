#ifndef _SERIALIO_
#define _SERIALIO_

#include "AP_Math.h"
#include "DCM.h"
#include "Sensor.h"

#define BAUDRATE 115200

class SerialIO
{
  public:
  
	SerialIO(){}
	
	void init()
	{
		Serial.begin(BAUDRATE);	
	}
	
	void print_sensor(Sensor& sensor)
	{
		Serial.print("[ Accel ] ");
		Serial.print(sensor.get_accel().x); Serial.print("  ");
		Serial.print(sensor.get_accel().y); Serial.print("  ");
		Serial.print(sensor.get_accel().z); Serial.print("\t");
		Serial.print("[ Gyro ] ");
		Serial.print(ToDeg(sensor.get_gyro().x)); Serial.print("  ");
		Serial.print(ToDeg(sensor.get_gyro().y)); Serial.print("  ");
		Serial.print(ToDeg(sensor.get_gyro().z)); Serial.print("\t");
	}
	
	void print_dcm(DCM& dcm)
	{
		Serial.print("[ DCM ] ");
		Serial.print(ToDeg(dcm.roll)); Serial.print("  ");
		Serial.print(ToDeg(dcm.pitch)); Serial.print("  ");
		Serial.print(ToDeg(dcm.yaw)); Serial.print("\n");
	}
};

#endif

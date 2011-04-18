#ifndef _SERIALIO_
#define _SERIALIO_

#include "AP_Math.h"
#include "DCM.h"
#include "Sensor.h"
#include "MotorCmd.h"

#define BAUDRATE 115200

class SerialIO
{
  public:
  
	SerialIO(){}

	union SERIAL_FLOAT{
		uint8_t data[4];
		float number;
	};

	void init()
	{
		Serial.begin(BAUDRATE);
	}
	
	void print_sensor(Sensor& sensor)
	{
		Serial.print("[Acc] ");
		Serial.print(sensor.get_accel().x); Serial.print(" ");
		Serial.print(sensor.get_accel().y); Serial.print(" ");
		Serial.print(sensor.get_accel().z); Serial.print(" ");
		Serial.print("[Gyr] ");
		Serial.print(ToDeg(sensor.get_gyro().x)); Serial.print(" ");
		Serial.print(ToDeg(sensor.get_gyro().y)); Serial.print(" ");
		Serial.print(ToDeg(sensor.get_gyro().z)); Serial.print(" ");
	}
	
	void print_dcm(DCM& dcm)
	{
		Serial.print("[DCM] ");
		Serial.print(ToDeg(dcm.roll)); Serial.print(" ");
		Serial.print(ToDeg(dcm.pitch)); Serial.print(" ");
		Serial.print(ToDeg(dcm.yaw)); Serial.print(" ");
	}
	
	void print_motor(MotorCmd& motor)
	{
		Serial.print("[PID] ");
		Serial.print(motor.control_roll);Serial.print(" ");
		Serial.print(motor.control_pitch);Serial.print(" ");
		Serial.print(motor.control_yaw);Serial.print(" ");
		Serial.print("[CMD] ");
		for(int i = 0; i < 4; i++){
			Serial.print(motor.OutputCmd[i]);Serial.print(" ");}
	}

	void print_pid_detail(MotorCmd& motor)
	{
		Serial.print(motor.pid_roll.P);Serial.print(" ");Serial.print(motor.pid_roll.I);Serial.print(" ");Serial.print(motor.pid_roll.D);Serial.print("    |    ");
		Serial.print(motor.pid_pitch.P);Serial.print(" ");Serial.print(motor.pid_pitch.I);Serial.print(" ");Serial.print(motor.pid_pitch.D);Serial.print("    |    ");
		Serial.print(motor.control_roll);Serial.print(" ");
		Serial.print(motor.control_pitch);Serial.print(" ");
		Serial.print(motor.control_yaw);Serial.print(" ");
	}

	void print_motor_cmdraw(MotorCmd& motor)
	{
		Serial.print("[RAW] ");
		for(int i = 0; i < 4; i++){
		Serial.print(APM_RC.InputCh(i)); Serial.print(" ");}
		Serial.print("[OTHER] ");
              Serial.print(motor.pilot_roll); Serial.print(" ");
             Serial.print(motor.pilot_pitch); Serial.print(" ");
              Serial.print(motor.pilot_yaw); Serial.print(" ");
                Serial.print(motor.pilot_throttle); Serial.print(" ");
	}
	
	void write_dcm(DCM& dcm)
	{
		uint8_t output_arr[9];
		union SERIAL_FLOAT sf;
		output_arr[0] = 'A';
		sf.number = ToDeg(dcm.roll);
		for(int i = 0; i < 4; i++)
			output_arr[i+1] = sf.data[i];
		sf.number = ToDeg(dcm.pitch);
		for(int i = 0; i < 4; i++)
			output_arr[i+5] = sf.data[i];
		Serial.write(output_arr, 9);
	}
};

#endif

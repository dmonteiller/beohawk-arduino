//      ART_Arduino.pde
//      Main program for running arduino board functionalities (sensor reading and filtering, serial I/O, etc).
//
//      Copyright (C) 2011 Sam (Yujia Zhai) <yujia.zhai@usc.edu>
//      Aerial Robotics Team, USC Robotics Society - http://www.uscrs.org - http://uscrs.googlecode.com
//
//      This program is free software; you can redistribute it and/or modify
//      it under the terms of the GNU General Public License as published by
//      the Free Software Foundation; either version 2 of the License, or
//      (at your option) any later version.
//      
//      This program is distributed in the hope that it will be useful,
//      but WITHOUT ANY WARRANTY; without even the implied warranty of
//      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//      GNU General Public License for more details.
//      
//      You should have received a copy of the GNU General Public License
//      along with this program; if not, please refer to the online version on
//      http://www.gnu.org/licenses/gpl.html, or write to the Free Software
//      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
//      MA 02110-1301, USA.

#include "AP_Math.h"
#include <AP_ADC.h>
#include "APM_RC.h"
#include "Sensor.h"
#include "DCM.h"
#include "SerialIO.h"
#include "Board.h"
#include "MotorCmd.h"

class Main
{
  public:
	Board board;
	SerialIO io;
	Sensor sensor;
	DCM dcm;
	AP_ADC adc;
	MotorCmd motor;
	
	Vector3f raw_gyro, raw_accel; // order: roll, pitch; yaw, x, y, z
	int raw_motor[4]; // order: roll, pitch, throttle, yaw
	
	Main(): sensor(adc), dcm(sensor), motor(sensor, dcm) {}
	
	void init()
	{
  		io.init();
		board.init();
		
		board.ledon(red);
		adc.init();
		APM_RC.Init();
		delay(100);
		sensor.init();
		
		board.ledoff(red);
	}
	
	void slowloop(float _interval)
	{
		//io.print_sensor(sensor);
		//io.print_dcm(dcm);
                //io.write_dcm(dcm);
                io.print_motor_detail(motor);
       		Serial.println("");
	}
	
	void fastloop(float _interval)
	{
		dcm.update_DCM(_interval);
                motor.getCommands();
                motor.processPID(_interval);
		motor.applyCommands();
	}
};

#define INTERVAL_SLOWLOOP 100
#define INTERVAL_FASTLOOP 20

long time_slowloop, time_fastloop;
Main m;

void setup()
{
	m.init();
	
	time_slowloop = millis();
	time_fastloop = millis();
}

void loop()
{
	if(millis() - time_slowloop > INTERVAL_SLOWLOOP)
	{
		float interval = (millis() - time_slowloop) / 1000.0;
		time_slowloop = millis();
		
		m.slowloop(interval);
	}
	
	if(millis() - time_fastloop > INTERVAL_FASTLOOP)
	{
		float interval = (millis() - time_fastloop) / 1000.0;
		time_fastloop = millis();
		
		m.fastloop(interval);
	}
}

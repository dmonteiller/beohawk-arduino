//      Sensor.h
//      IMU reading support for ART_Arduino.
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

#ifndef _SENSOR_
#define _SENSOR_

#include <WProgram.h>

#include "AP_Math.h"
#include "AP_ADC.h"

class Sensor
{
  private:
	AP_ADC& adc;
	Vector3f accel;
	Vector3f gyro;

	static int sensor_id[6];
	static int sensor_sign[6];
	int sensor_offset[6];

	static const float gyro_gain_x = 0.4, gyro_gain_y = 0.41, gyro_gain_z = 0.41;
	
  public:
	Sensor(AP_ADC& _adc): adc(_adc) {}
	
	void init()
	{	
		for(int i = 0; i < 6; i++)
			adc.Ch(sensor_id[i]);

		delay(50);
	
		long _average[6] = {0, 0, 0, 0, 0, 0};

		for(int i = 0; i < 50; i++)
			for (int j = 0; j < 3; j++)
			{
				_average[j] += adc.Ch(sensor_id[j]);
				delay(30);
			}

		for(int i = 0; i < 3; i++)
			sensor_offset[i] = _average[i] / 50;
		
		for(int i = 0; i < 10; i++)
			for (int j = 3; j < 6; j++)
			{
				_average[j] += adc.Ch(sensor_id[j]);
				delay(20);
			}

		for(int i = 3; i < 6; i++)
			sensor_offset[i] = _average[i] / 10;
		sensor_offset[5] = 2010;
	}
	
	bool update()
	{
		float temp[6];
		for(int i = 0; i < 6; i++)
		    if (sensor_sign[i] < 0) 
				temp[i] = sensor_offset[i] - adc.Ch(sensor_id[i]);
			else
				temp[i] = adc.Ch(sensor_id[i]) - sensor_offset[i];

		gyro.x = ToRad(gyro_gain_x) * temp[0];
		gyro.y = ToRad(gyro_gain_y) * temp[1];
		gyro.z = ToRad(gyro_gain_z) * temp[2];
		accel.x = temp[3];
		accel.y = temp[4];
		accel.z = temp[5];
	}

	Vector3f get_gyro() { return gyro; }
	Vector3f get_accel() { return accel; }
};

int Sensor::sensor_id[6] = {1, 2, 0, 4, 5, 6};
int Sensor::sensor_sign[6] = {1,-1,-1, 1,-1,-1};

#endif

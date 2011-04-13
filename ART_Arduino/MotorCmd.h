#ifndef _MOTOR_
#define _MOTOR_

#include "APM_RC.h"

class MotorCmd
{
  private:
	struct PID
	{
		float Kp, Ki, Kd;
		float P, I, D;
		
		float P_prev;
		
		PID(int _Kp, int _Ki, int _Kd): Kp(_Kp), Ki(_Ki), Kd(_Kd), P(0), I(0), D(0), P_prev(0) {}
		
		process(float value_obj, float value_ref, int Dt)
		{
			P = value_obj - value_ref;
			I += P * Dt;
			D = (P - P_prev) / Dt;
			
			p_prev = P;
			
			return (Kp * P + Ki * I + Kd * D);
		}
	}
	
	APM_RC& rc;
	Sensor& sensor;
	
	int old_rc_reading[4];
	
	struct PID pid_roll, pid_pitch, pid_yaw;
	
	float control_roll, control_pitch, control_yaw;
	int pilot_roll, pilot_pitch, pilot_yaw, pilot_throttle;
	int OutputCmd[4]; // motor 1, 2, 3, 4
	
	static const mid_throttle = 1200;
	static const mid_channel = 1500;
	static const min_channel = 1100;
	static const max_channel = 2000;
	static const threshold_change = 40;

	bool serial_pilot_mode;
	bool motors_armed;
	
  public:	
	MotorCmd(APM_RC& _rc, Sensor& _sensor): rc(_rc), sensor(_sensor),
		controlroll(0), controlpitch(0), controlyaw(0),
		serial_pilot_mode(true), motors_armed(false)
	{
		pid_roll.Kp = 3.0;
		pid_roll.Ki = 0.0;
		pid_roll.Kd = 1.0;
		pid_pitch.Kp = 3.0;
		pid_pitch.Ki = 0.0;
		pid_pitch.Kd = 1.0;
		pid_yaw.Kp = 2.0;
		pid_yaw.Ki = 0.0;
		pid_yaw.Kd = 0.0;
		
		for(int i = 0; i < 4; i++)
			old_rc_reading[i] = min_channel;
	}
	
	void getCommands(){
		if(serial_pilot_mode){
			
		}
		else
		{
			int temp;
			
			temp = APM_RC.InputCh(0);
			pilot_roll = 0.1 * (constrain(temp, old_rc_reading[0] - threshold_change, old_rc_reading[0] + threshold_change) - mid_channel);
			old_rc_reading[0] = temp;
			
			temp = APM_RC.InputCh(1);
			pilot_pitch = - 0.1 * (constrain(temp, old_rc_reading[1] - threshold_change, old_rc_reading[1] + threshold_change) - mid_channel);
			old_rc_reading[1] = temp;
			
			temp = APM_RC.InputCh(3);
			pilot_yaw = 0.1 * (constrain(temp, old_rc_reading[3] - threshold_change, old_rc_reading[3] + threshold_change) - mid_channel);
			old_rc_reading[3] = temp;
			
			temp = APM_RC.InputCh(2);
			pilot_throttle = constrain(temp, old_rc_reading[2] - threshold_change, old_rc_reading[2] + threshold_change);
			old_rc_reading[2] = temp;
			
			if(old_rc_reading[3] < 1200)
				motors_armed = false;
			else if(old_rc_reading[3] > 1800)
				motors_armed = true;
		}
	}
	
	void processPID(int Dt)
	{
		control_roll = pid_roll.process(pilot_roll, ToDeg(sensor.get_gyro().x) ,Dt);
		control_pitch = pid_pitch.process(pilot_pitch, ToDeg(sensor.get_gyro().y) ,Dt);
		control_yaw = pid_yaw.process(pilot_yaw, ToDeg(sensor.get_gyro().z) ,Dt);
	}
	
	void applyCommands()
	{
		if(motors_armed)
		{
			OutputCmd[0] = constrain(pilot_throttle + control_roll + control_pitch - control_yaw, 1100,2000);
			OutputCmd[1] = constrain(pilot_throttle + control_roll - control_pitch + control_yaw, 1100,2000);
			OutputCmd[2] = constrain(pilot_throttle - control_roll - control_pitch - control_yaw, 1100,2000);
			OutputCmd[3] = constrain(pilot_throttle - control_roll + control_pitch + control_yaw, 1100,2000);
			APM_RC.OutputCh(0, OutputCmd[0]);
			APM_RC.OutputCh(1, OutputCmd[1]);
			APM_RC.OutputCh(2, OutputCmd[2]);
			APM_RC.OutputCh(3, OutputCmd[3]);
		} 
		else
		{
			APM_RC.OutputCh(0,1100);
			APM_RC.OutputCh(1,1100);
			APM_RC.OutputCh(2,1100);
			APM_RC.OutputCh(3,1100);
		}
	}
};

#endif

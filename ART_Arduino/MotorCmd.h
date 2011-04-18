#ifndef _MOTOR_
#define _MOTOR_

#include "APM_RC.h"

class MotorCmd
{
  public:
	struct PID
	{
		float Kp, Ki, Kd;
		float P, I, D;
		float threshold;
		float P_prev;

		PID() {}
		PID(float _Kp, float _Ki, float _Kd): Kp(_Kp), Ki(_Ki), Kd(_Kd), P(0), I(0), D(0), P_prev(0), threshold(100) {}
		
		float process(float value_obj, float value_ref, float Dt)
		{
			P = constrain(value_obj - value_ref, -threshold, threshold);
			I += P * Dt;
			I = constrain(I, -20, 20);
			D = (P - P_prev) / Dt;
			P_prev = P;

			return (Kp * P + Ki * I + Kd * D);
		}
		
		/** use the D result from DCM **/
		float process_dcm(float value_obj, float value_ref, float Dt, float D_dcm)
		{
			P = constrain(value_obj - value_ref, -threshold, threshold);
			I += P * Dt;
			I = constrain(I, -20, 20);
			D = D_dcm;
			P_prev = P;

			return (Kp * P + Ki * I + Kd * D);
		}
	};
	
	DCM& dcm;
	Sensor& sensor;
	
	int old_rc_reading[4];
	
	struct PID pid_roll, pid_pitch, pid_yaw;
	
	float control_roll, control_pitch, control_yaw;
	float pilot_roll, pilot_pitch, pilot_yaw, pilot_throttle;

	int OutputCmd[4]; // motor 1, 2, 3, 4
	
	static const int mid_throttle = 1200;
	static const int mid_channel = 1500;
	static const int min_channel = 1100;
	static const int max_channel = 2000;
	static const int threshold_change = 40;

	bool serial_pilot_mode;
	bool motors_armed;
	
  public:	
	MotorCmd(Sensor& _sensor, DCM& _dcm): sensor(_sensor), dcm(_dcm),
		control_roll(0), control_pitch(0), control_yaw(0),
		serial_pilot_mode(false), motors_armed(false)
	{
		pid_roll.Kp = 3.0;
		pid_roll.Ki = 0.0;
		pid_roll.Kd = 1.0;
		pid_roll.threshold = 60;
		pid_pitch.Kp = 3.0;
		pid_pitch.Ki = 0.0;
		pid_pitch.Kd = 1.0;
		pid_pitch.threshold = 60;
		pid_yaw.Kp = 2.0;
		pid_yaw.Ki = 0.0;
		pid_yaw.Kd = 0.0;
		pid_yaw.threshold = 80;
				
		/** calibrate the motors **/
		APM_RC.OutputCh(0,1100);
		APM_RC.OutputCh(1,1100);
		APM_RC.OutputCh(2,1100);
		APM_RC.OutputCh(3,1100);
		delay(500);
		APM_RC.OutputCh(0,2000);
		APM_RC.OutputCh(1,2000);
		APM_RC.OutputCh(2,2000);
		APM_RC.OutputCh(3,2000);
		delay(500);  
		APM_RC.OutputCh(0,1100);
		APM_RC.OutputCh(1,1100);
		APM_RC.OutputCh(2,1100);
		APM_RC.OutputCh(3,1100);
		delay(500);
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
			pilot_yaw = 1.0 * (constrain(temp, old_rc_reading[3] - threshold_change, old_rc_reading[3] + threshold_change) - mid_channel);
			old_rc_reading[3] = temp;

			temp = APM_RC.InputCh(2);
			pilot_throttle = temp;
			old_rc_reading[2] = temp;
		}
	}
	
	void processPID(float Dt)
	{
		control_roll = pid_roll.process_dcm(pilot_roll, ToDeg(dcm.roll) ,Dt, - ToDeg(dcm.omega_integ_corr.x));
		control_pitch = pid_pitch.process_dcm(pilot_pitch, ToDeg(dcm.pitch) ,Dt, - ToDeg(dcm.omega_integ_corr.y));
		control_yaw = pid_yaw.process_dcm(pilot_yaw, 0.5 * sensor.get_gyro().z, Dt, - ToDeg(dcm.omega_integ_corr.z));
	}
	
	void applyCommands()
	{
		if(pilot_throttle < mid_throttle)
		{
			control_yaw = 0;
			if(old_rc_reading[3] < 1200)
    				motors_armed = false;
	    		else if(old_rc_reading[3] > 1800)
				motors_armed = true;
		}
		if(motors_armed)
		{
			OutputCmd[0] = constrain(pilot_throttle + control_roll + control_pitch - control_yaw, min_channel, max_channel);
			OutputCmd[1] = constrain(pilot_throttle + control_roll - control_pitch + control_yaw, min_channel, max_channel);
			OutputCmd[2] = constrain(pilot_throttle - control_roll - control_pitch - control_yaw, min_channel, max_channel);
			OutputCmd[3] = constrain(pilot_throttle - control_roll + control_pitch + control_yaw, min_channel, max_channel);

			APM_RC.OutputCh(0, OutputCmd[0]);
			APM_RC.OutputCh(1, OutputCmd[1]);
			APM_RC.OutputCh(2, OutputCmd[2]);
			APM_RC.OutputCh(3, OutputCmd[3]);
		} 
		else
		{
			APM_RC.OutputCh(0, min_channel);
			APM_RC.OutputCh(1, min_channel);
			APM_RC.OutputCh(2, min_channel);
			APM_RC.OutputCh(3, min_channel);
		}
	}
};

#endif

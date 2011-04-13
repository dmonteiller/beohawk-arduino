#ifndef _DCM_
#define _DCM_

#include "AP_Math.h"
#include "Sensor.h"

class DCM
{
  private:
	Sensor& sensor;
	Matrix3f dcm_matrix;
	Vector3f accel_vector;
	Vector3f gyro_vector;
	Vector3f omega_P;
	Vector3f omega_I;
	Vector3f omega_integ_corr;
	Vector3f omega;
	Vector3f error_roll_pitch;
	
	static const float KpDCM_rollpitch = .002;
	static const float KiDCM_rollpitch = .0000005;
	static const float KpDCM_yaw = 1.5;
	static const float KiDCM_yaw = .00005;
	
  public:
	float roll, pitch, yaw;
		
  public:
	DCM(Sensor& _sensor): sensor(_sensor), dcm_matrix(1, 0, 0, 0, 1, 0, 0, 0, 1) {}
	
	void update_DCM(float _Dt)
	{
		sensor.update();
		gyro_vector = sensor.get_gyro();
		accel_vector = sensor.get_accel();
	
		matrix_update(_Dt);
		normalize();
		drift_correction();
		euler_angles();
	}
	
  private:
	void matrix_update(float _Dt)
	{
		Matrix3f update_matrix;
		Matrix3f temp_matrix;

		omega_integ_corr = gyro_vector + omega_I;
		omega = omega_integ_corr + omega_P;
		
		update_matrix.a.x = 0;
		update_matrix.a.y = - _Dt * omega.z;
		update_matrix.a.z = _Dt * omega.y;
		update_matrix.b.x = _Dt * omega.z;
		update_matrix.b.y = 0;
		update_matrix.b.z = - _Dt * omega.x;
		update_matrix.c.x = - _Dt * omega.y;
		update_matrix.c.y = _Dt * omega.x;
		update_matrix.c.z = 0;
	 
		temp_matrix = dcm_matrix * update_matrix;
		dcm_matrix = dcm_matrix + temp_matrix;
	}
	
	
	void normalize()
	{
		Vector3f temporary[3];
		float error = dcm_matrix.a * dcm_matrix.b;
		temporary[0] = dcm_matrix.b;
		temporary[1] = dcm_matrix.a;
		temporary[0] = dcm_matrix.a - (temporary[0] * (0.5 * error));
		temporary[1] = dcm_matrix.b - (temporary[1] * (0.5 * error));
		temporary[2] = temporary[0] % temporary[1];
		
		dcm_matrix.a = temporary[0] * (.5 * (3 - (temporary[0] * temporary[0])));
		dcm_matrix.b = temporary[1] * (.5 * (3 - (temporary[1] * temporary[1])));
		dcm_matrix.c = temporary[2] * (.5 * (3 - (temporary[2] * temporary[2])));
	}
	
	void drift_correction()
	{
		error_roll_pitch = accel_vector % dcm_matrix.c;
		omega_P = error_roll_pitch * (KpDCM_rollpitch);
		omega_I += error_roll_pitch * (KiDCM_rollpitch);
	}
	
	void euler_angles()
	{
		pitch = - asin(dcm_matrix.c.x);
		roll = atan2(dcm_matrix.c.y, dcm_matrix.c.z);
		yaw = atan2(dcm_matrix.b.x, dcm_matrix.a.x);
	}
};

#endif

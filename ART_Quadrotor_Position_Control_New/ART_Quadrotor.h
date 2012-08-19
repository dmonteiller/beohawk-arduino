#include <WProgram.h>

#define GRAVITY 408 // 1g on the accelerometer
#define BAUDRATE 115200
#define MAGNETOMETER 0

#define LEDYELLOW 36
#define LEDRED 35
#define LEDGREEN 37
#define SWITCH1 41
#define SWITCH2 40

#define MINTHROTTLE 1200
#define MIDCHANNEL 1500

#define Gyro_Gain_X 0.4  //X axis Gyro gain
#define Gyro_Gain_Y 0.41 //Y axis Gyro gain
#define Gyro_Gain_Z 0.41 //Z axis Gyro gain

// Define macros //
#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

// PID constants
float Kproll = 1.0;
float Kiroll = 0.2;
float Kdroll = 0.6;
float Kppitch = 1.0;
float Kipitch = 0.2;
float Kdpitch = 0.6;
float Kpyaw = 1.0;
float Kiyaw = 0.0;
float Kdyaw = 0.6;
float Kpaltitude = 1.0; 
float Kialtitude = 0.4;  
float Kdaltitude = 0.4;  



//PID Max Term's
float pitchRollIMax = 50;
float altitudeIMax = 200;

// Define vars //
float loopDt = 0.02; // This will be changed per loop
float loopDt_slow = 0.02;
long timer1 = 0;
long timer2 = 0;
long timer3 = 0;
long telemetryTimer = 0;
long compassReadTimer = 0;
long sonarTimer = 0;
long otherTimer = 0;
long timeStep = 0;
long landingTime = 0;
long counter = 0;
long altHoldStart = 0;
long altHoldDelay = 0;

float previousActualAltitude = 17;
float previousSonarAltitude = 17;
float usedAltitude = 17;
int motorsArmed = 0;
float desiredAltitude = 0;
float sonarAltitude = 0;
int sonarData[8];
float pressureAltitude = 0;
float groundPressureAltitude = 0;
float actualAltitude = 17.0; 
byte currentSonarData = 0;
int landingAltitude = 0;
int altitudeThrottle = 0;
int holdingAltitude = 0;
int flag = 0;
float desiredXPosition;
float desiredYPosition;
float desiredPitch;
float desiredRoll;
float desiredYaw;
float desiredYawChange;
float laserYaw;
float laserXPosition;
float laserYPosition;

int controlRoll = 0;
int controlPitch = 0;
int controlYaw = 0;
int controlAltitude = 0;


float rollError;
float rollErrorOld;
float rollI;
float rollD;
float pitchError;
float pitchErrorOld;
float pitchI;
float pitchD;
float yawError;
float yawErrorOld;
float yawI;
float yawD;
float altitudeError;
float altitudeErrorOld;
float altitudeI;
float altitudeD;

// ADC storage
int ADC_Ch[6];
int ADC_Offset[6];

int accelOffset[3] = {2073,2056,2010};
int gyroOffset[3] = {1659,1618,1673};

float Omega[3]= {0,0,0};

// Transmitter Data Storage
int RCInput[8];
int pilotRoll = 0;
int pilotRollOld = 0;
int pilotPitch = 0;
int pilotPitchOld = 0;
int pilotYaw = 0;
int pilotYawOld = 0;
int pilotXChange;
int pilotYChange;
int pilotThrottle = 0;
int throttle = 1200;
int holdingThrottle = 0;
long rosTimer = 0;

// DCM PID Values (these will be hard code initialized)
float KpDCM_rollpitch = .002;
float KiDCM_rollpitch = .0000005;
float KpDCM_yaw = 1.5;
float KiDCM_yaw = .00005;

// Vehicle orientation angles
float roll;
float pitch;
float yaw;

int motor[4];

//State Machine
bool isLanding = false;
bool isManualControl = true;
bool holdingPosition = false;
bool altHold = false;
bool positionHold = false;


// ROS msg packet
union PositionPacket
{
  struct
  {
    long x;
    long y;
    long theta;
  };
  unsigned char data[12];
} pos_pack;

/*
union AttitudePacket
{
    struct
    {
        float roll;
        float pitch;
        float yaw;
    };
     unsigned  char data[12];
} attitude_pack;
*/







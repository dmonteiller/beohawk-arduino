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
float Kproll = 1.5;
float Kiroll = 3.0;
float Kdroll = 1.5;
float Kppitch = 1.5;
float Kipitch = 3.0;
float Kdpitch = 1.5;
float Kpyaw = 2.0;
float Kiyaw = 0.0;
float Kdyaw = 0.0;
float Kpaltitude = 1.5;  //2
float Kialtitude = 1.5;  //0.1
float Kdaltitude = 1.6;  //1.2

// Define vars //
float loopDt = 0.02; // This will be changed per loop
long timer = 0;
long telemetryTimer = 0;
long compassReadTimer = 0;

int motorsArmed = 0;
int desiredAltitude = 0;
int sonarAltitude = 0;
int landingAltitude = 0;
bool holdingAltitude = false;
bool isLanding = false;
bool isManualControl = false;
long landingTime = 0;

float controlRoll = 0;
float controlPitch = 0;
float controlYaw = 0;
float controlAltitude = 0;

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
int pilotThrottle = 0;
int throttle = 0;

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


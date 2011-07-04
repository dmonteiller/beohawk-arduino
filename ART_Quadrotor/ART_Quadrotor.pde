#include <Wire.h>
#include <APM_RC.h>
#include <APM_ADC.h>
#include <APM_Compass.h>
#include <APM_BMP085.h>
#include <GPS_NMEA.h>
#include "ART_Quadrotor.h"

void setup() {
  Serial.begin(BAUDRATE);

  pinMode(LEDYELLOW,OUTPUT);
  pinMode(LEDRED,OUTPUT);
  pinMode(LEDGREEN,OUTPUT);
  pinMode(SWITCH1,INPUT);
  pinMode(SWITCH2,INPUT);  

  APM_RC.Init();
  APM_RC.OutputCh(0,1100);
  APM_RC.OutputCh(1,1100);
  APM_RC.OutputCh(2,1100);
  APM_RC.OutputCh(3,1100);
  delay(1000);
  /*APM_RC.OutputCh(0,2000);
  APM_RC.OutputCh(1,2000);
  APM_RC.OutputCh(2,2000);
  APM_RC.OutputCh(3,2000);*/
  delay(1000);  
  APM_RC.OutputCh(0,1100);
  APM_RC.OutputCh(1,1100);
  APM_RC.OutputCh(2,1100);
  APM_RC.OutputCh(3,1100);
  delay(1000);  

  APM_ADC.Init();
  //APM_Compass.Init();
  //GPS.Init();
  delay(100);  

  // Calibrate Gyros
  digitalWrite(LEDRED,HIGH);

  float aux_float[3];
  
  ADC_Offset[0] = gyroOffset[0];
  ADC_Offset[1] = gyroOffset[1];
  ADC_Offset[2] = gyroOffset[2];  
  ADC_Offset[3] = accelOffset[0];
  ADC_Offset[4] = accelOffset[1];
  ADC_Offset[5] = accelOffset[2];

  readADC();
  delay(20);

  long averageGyro[3] = {
    0,0,0      };

  for ( int i = 0 ; i < 100 ; i++ ) {
    readADC();
    for ( int j = 0 ; j < 3 ; j++ ) {
      averageGyro[j] += ADC_Ch[j];
    }
    delay(30);
  }
  for ( int i = 0 ; i < 3 ; i++ ) {
    ADC_Offset[i] = averageGyro[i]/100;    
  }

  digitalWrite(LEDRED,LOW);
  
  calibrateLevel();

  timer = millis();
  compassReadTimer = millis();  
  telemetryTimer = millis();
}

void loop() {

  if ( millis() - timer > 10 ) { // timer at 100 Hz

    loopDt = (millis()-timer)/1000.0;
    timer = millis();
    
    getMeasurements(); 
    
    processPicoITXSerial();
    
    // Read RC receiver
    for ( int i = 0 ; i < 4 ; i++ ) {
      RCInput[i] = radioFilter(APM_RC.InputCh(i),RCInput[i]);
    }

    for ( int i = 4 ; i < 8 ; i++ ) {
      RCInput[i] = APM_RC.InputCh(i);
    }

    pilotRollOld = pilotRoll;
    pilotRoll = 0.1*(RCInput[0]-MIDCHANNEL);
    pilotPitchOld = pilotPitch;
    pilotPitch = -0.1*(RCInput[1]-MIDCHANNEL);
    pilotThrottle = RCInput[2];
    pilotYawOld = pilotYaw;
    pilotYaw = 1.0*(RCInput[3]-MIDCHANNEL);

    PIDControl();

    if ( pilotThrottle < 1200 && motorsArmed == 0 ) {   //2nd condition makes it so this loop will be skipped if armed using computer instead of RC
      controlYaw = 0;
      rollI = 0;
      pitchI = 0;
      if ( RCInput[3] > 1800 ) {
        motorsArmed = 1;
      }
      if ( RCInput[3] < 1200 ) {
        motorsArmed = 0;
        isLanding = false;
      }
    }
    
    if ( RCInput[4] < 1500 ) {
      if ( holdingAltitude == false ) {
        altitudeI = 0;
        holdingAltitude = true;
        isManualControl = false;
        isLanding = false;        
        digitalWrite(LEDYELLOW,HIGH);        
      } 
      desiredAltitude = desiredAltitude*.8 + constrain(map(pilotThrottle,1200,1800,15,200),15,200)*.2;
      throttle = 1400;      
    } else {
      if (holdingAltitude == true && !isCompControl) {   //When given a desiredAltitude of 0 by comp it will enter this loop and land from sonarAltitude
        holdingAltitude = false;
        throttle = 1400;
        isLanding = true;
        isManualControl = false;        
        landingAltitude = sonarAltitude;
        landingTime = millis();
      } else if (isLanding) {
        throttle = 1400;
        desiredAltitude = constrain(landingAltitude - ((millis() - landingTime)/75),0,200);
        if (sonarAltitude < 18) {
          isLanding = false;
          holdingAltitude = false;
          motorsArmed = 0;
        }
      } else if (!isCompControl) {
        if ( isManualControl ) {
          throttle = pilotThrottle;
        } else if ( pilotThrottle < 1150 ) {
          isManualControl = true;
        } else {
          throttle = 1100;
        }        
        controlAltitude = 0; 
      }
      digitalWrite(LEDYELLOW,LOW);
    }

    if ( motorsArmed == 1 ) {
      motor[0] = constrain(throttle+controlRoll+controlPitch-controlYaw+controlAltitude,1100,2000);
      motor[1] = constrain(throttle+controlRoll-controlPitch+controlYaw+controlAltitude,1100,2000);
      motor[2] = constrain(throttle-controlRoll-controlPitch-controlYaw+controlAltitude,1100,2000);
      motor[3] = constrain(throttle-controlRoll+controlPitch+controlYaw+controlAltitude,1100,2000);
      APM_RC.OutputCh(0,motor[0]);
      APM_RC.OutputCh(1,motor[1]);
      APM_RC.OutputCh(2,motor[2]);
      APM_RC.OutputCh(3,motor[3]);
    } 
    else {
      APM_RC.OutputCh(0,1100);
      APM_RC.OutputCh(1,1100);
      APM_RC.OutputCh(2,1100);
      APM_RC.OutputCh(3,1100);
    }

  }
  
  if ( millis() - telemetryTimer > 200 ) {
    telemetryTimer = millis();
    sendTestData();
  }

}

void PIDControl() {
  rollError = constrain(pilotRoll - ToDeg(roll),-60,60);

  rollI += rollError*loopDt;
  rollI = constrain(rollI,-20,20);

  rollD = ToDeg(-Omega[0]);

  controlRoll = Kproll*rollError + Kiroll*rollI + Kdroll*rollD;
  
  ///////////////////////////////////////

  pitchError = constrain(pilotPitch - ToDeg(pitch),-60,60);

  pitchI += pitchError*loopDt;
  pitchI = constrain(pitchI,-20,20);

  pitchD = ToDeg(-Omega[1]);

  controlPitch= Kppitch*pitchError + Kipitch*pitchI + Kdpitch*pitchD;

/////////////////////////////////////////////

  yawError = constrain(pilotYaw - .5*readADCCorrected(2),-80,80);

  yawI += yawError*loopDt;
  yawI = constrain(yawI,-20,20);

  //yawD = readADCCorrected(2);

  controlYaw= Kpyaw*yawError + Kiyaw*yawI + Kdyaw*yawD;
  
/////////////////////////////////////////////

  altitudeError = desiredAltitude-sonarAltitude;

  altitudeI += altitudeError*loopDt;
  altitudeI = constrain(altitudeI,-1000,1000);
  
  altitudeD = (altitudeError - altitudeErrorOld)/loopDt;
  altitudeErrorOld = altitudeError;

  controlAltitude = Kpaltitude*altitudeError + Kialtitude*altitudeI + Kdaltitude*altitudeD;  
  
  
}

// Maximum slope filter for radio inputs... (limit max differences between readings)
int radioFilter(int ch, int ch_old)
{
  int diff_ch_old;

  if (ch_old==0)      // ch_old not initialized
    return(ch);
  diff_ch_old = ch - ch_old;      // Difference with old reading
  if (diff_ch_old<0)
  {
    if (diff_ch_old<-40)
      return(ch_old-40);        // We limit the max difference between readings
  }
  else
  {
    if (diff_ch_old>40)    
      return(ch_old+40);
  }
  return(ch);
}

void getMeasurements() {
  readADC();
  if (MAGNETOMETER == 1) {
    if (millis() - compassReadTimer >= 100)  // Read compass data at 10Hz...
    {
      compassReadTimer = millis();
      APM_Compass.Read();     // Read magnetometer
      APM_Compass.Calculate(roll,pitch);  // Calculate heading
    }
  }
  Matrix_update();
  Normalize();
  Drift_correction();
  Euler_angles();
  
  sonarAltitude = sonarAltitude*.9 + constrain(analogRead(A6),15,500)*.1;
}

void calibrateLevel() {
  digitalWrite(LEDRED,HIGH);

  // Calibrate accelOffset
  int average_x_offset = 0;
  int average_y_offset = 0;

  for ( int i = 0 ; i < 10 ; i++ ) {
    readADC();
    average_x_offset += ADC_Ch[3];
    average_y_offset += ADC_Ch[4];
    delay(20);
  }

  accelOffset[0] = average_x_offset/10;
  accelOffset[1] = average_y_offset/10;

  ADC_Offset[3] = accelOffset[0];
  ADC_Offset[4] = accelOffset[1];
  ADC_Offset[5] = accelOffset[2];

  digitalWrite(LEDRED,LOW);
}



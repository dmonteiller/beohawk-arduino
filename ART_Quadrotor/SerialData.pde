
#define MSG_POSITIONAL_ERROR 1
#define MSG_ARM_MOTORS 2
#define MSG_DESIRED_ALTITUDE 3

void processPicoITXSerial() {
  // Create and send serial message containing Attitude Info to PicoITX
  id = 4;
  data[0] = roll;
  data[1] = pitch;
  data[2] = yaw;
  data[3] = sonarAltitude;
  checkSum = id ^ data[0] ^ data[1] ^ data[2] ^ data[3];
  //Serial.write(id);
  //Serial.write(data, 4);
  //Serial.write(checkSum);

  // Read serial message from PicoITX
  num = Serial.available();
  timer = millis();
  while (num >= 6 && millis() - timer < 12) {
    id = Serial.read();
    data[0] = Serial.read();
    data[1] = Serial.read();
    data[2] = Serial.read();
    data[3] = Serial.read();
    checkSum = Serial.read();
    Serial.print("Read: ");
    Serial.print(id,DEC);Serial.print('\t');
    Serial.print(data[0]);Serial.print('\t');
    Serial.print(data[1]);Serial.print('\t');
    Serial.print(data[2]);Serial.print('\t');
    Serial.print(data[3]);Serial.print('\t');
    Serial.print(checkSum);Serial.println('\t');
    if (true) { //(id ^ data[0] ^ data[1] ^ data[2] ^ data[3]) == checkSum) {
      switch (id) {
      case MSG_POSITIONAL_ERROR: 
        xError = data[0];     // Positional Error
        yError = data[1];
        // Reset serial message handlers
        id = 0;
        checkSum = 0;
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;        
        break;
      case MSG_ARM_MOTORS: 
        motorsArmed = data[0];     // Arm Motors
        // Reset serial message handlers
        id = 0;
        checkSum = 0;
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;        
        break;
      case MSG_DESIRED_ALTITUDE: 
        desiredAltitude = data[0];     // Desired Altitude
        Serial.print("Desired altitude: "); Serial.println(desiredAltitude);
        if (desiredAltitude <= 0) {
          if ( isManualControl == false ) {
            isLanding = true;
            landingAltitude = sonarAltitude;
            landingTime = millis();
          }
        } 
        else {
          isManualControl = false;
          throttle = 1400;                      
        }
        holdingAltitude = true;
        // Reset serial message handlers
        id = 0;
        checkSum = 0;
        data[0] = 0;
        data[1] = 0;
        data[2] = 0;
        data[3] = 0;        
        break; 
      }
    }
    num = Serial.available();
  }
}  

void sendTestData() {
  /* Serial.print(controlRoll);Serial.print(", ");
   Serial.print(controlPitch);Serial.print(", ");
   Serial.print(controlYaw);Serial.print(",  "); */
  Serial.print(controlAltitude);
  Serial.print(",  ");  
  Serial.print(analogRead(A6));
  Serial.print(",  ");  
  Serial.print(desiredAltitude);
  Serial.print(",  ");  
  Serial.print(motor[0]);
  Serial.print(", ");
  Serial.print(motor[1]);
  Serial.print(", ");
  Serial.print(motor[2]);
  Serial.print(", ");
  Serial.print(motor[3]);
  Serial.print(", ");

  Serial.print(ToDeg(roll)); 
  Serial.print(", ");
  Serial.print(ToDeg(pitch)); 
  Serial.print(", ");
  Serial.print(ToDeg(yaw)); 
  Serial.print(", ");

  /* Serial.print(readADCCorrected(0)); Serial.print(", ");
   Serial.print(readADCCorrected(1)); Serial.print(", ");
   Serial.print(readADCCorrected(2)); Serial.print(", ");
   Serial.print(readADCCorrected(3)); Serial.print(", ");
   Serial.print(readADCCorrected(4)); Serial.print(", ");
   Serial.print(readADCCorrected(5)); Serial.print(", ");*/

  Serial.println(" ");

}



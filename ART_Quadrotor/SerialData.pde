

void sendTestData() {
/* Serial.print(controlRoll);Serial.print(", ");
 Serial.print(controlPitch);Serial.print(", ");
 Serial.print(controlYaw);Serial.print(",  "); */
 Serial.print(controlAltitude);Serial.print(",  ");  
 Serial.print(analogRead(A6));Serial.print(",  ");  
 Serial.print(desiredAltitude);Serial.print(",  ");  
 Serial.print(motor[0]);Serial.print(", ");
 Serial.print(motor[1]);Serial.print(", ");
 Serial.print(motor[2]);Serial.print(", ");
 Serial.print(motor[3]);Serial.print(", ");

 Serial.print(ToDeg(roll)); Serial.print(", ");
 Serial.print(ToDeg(pitch)); Serial.print(", ");
 Serial.print(ToDeg(yaw)); Serial.print(", ");
 
/* Serial.print(readADCCorrected(0)); Serial.print(", ");
 Serial.print(readADCCorrected(1)); Serial.print(", ");
 Serial.print(readADCCorrected(2)); Serial.print(", ");
 Serial.print(readADCCorrected(3)); Serial.print(", ");
 Serial.print(readADCCorrected(4)); Serial.print(", ");
 Serial.print(readADCCorrected(5)); Serial.print(", ");*/
 
 Serial.println(" ");
  
}

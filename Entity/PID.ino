// -90 to turn left
// 0 to go forward
// 90 to turn right
int calculateNewSetpoint(int angle){
  int newSetpoint = Setpoint;
  newSetpoint += angle;
  if(newSetpoint < -180)
    newSetpoint += 360;
  if(newSetpoint > 180)
    newSetpoint -= 360;
  return newSetpoint;
}

void forwardPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu){
  readPosition(bno, event, mpu, 'B');
  xBNOKalmanFilter();
  Input = xBNOXe;
  leftPID.Compute();  //Gets an output
  rightPID.Compute();
  analogWrite(motorR2, velGenDer + rightOutput);
  analogWrite(motorL2, slowGo() + leftOutput);
  analogWrite(motorR1, 0);
  analogWrite(motorL1, 0);
}   

void alignCenterPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu){
  double startTime = millis();
  double endTime = 0;
  readPosition(bno, event, mpu, 'B');
  while(endTime-startTime < 5000 || (rightOutput==0 && leftOutput==0)){
     bno.getEvent(&event);
     xBNOKalmanFilter();
     Input = xBNOXe;
     leftPID.Compute();  //Gets an output
     rightPID.Compute();
     Serial.print(leftOutput);
     Serial.print("\t");
     Serial.println(rightOutput);
     if(rightOutput == 0){
       analogWrite(motorL2, leftOutput*5);
       analogWrite(motorR2, 0);
       analogWrite(motorR1, leftOutput*5);
       analogWrite(motorL1, 0);     
     }
     else{
       analogWrite(motorR2, rightOutput*5);
       analogWrite(motorL2, 0);
       analogWrite(motorR1, 0);
       analogWrite(motorL1, rightOutput*5);        
     }
     ledsPID();
     readPosition(bno, event, mpu, 'B');
     endTime = millis();
  }
  blinkingLEDS();
}

void turnLeftPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu){
  double startTime = millis();
  double endTime = 0;
  readPosition(bno, event, mpu, 'B');
  while(endTime-startTime < 7000 || (rightOutput==0 && leftOutput==0)){
     bno.getEvent(&event);
     xBNOKalmanFilter();
     Input = xBNOXe;
     leftPID.Compute();  //Gets an output
     rightPID.Compute();
     Serial.print(leftOutput);
     Serial.print("\t");
     Serial.println(rightOutput);
     if(rightOutput == 0){
       analogWrite(motorL2, leftOutput*0.45);
       analogWrite(motorR2, 0);
       analogWrite(motorR1, leftOutput*0.45);
       analogWrite(motorL1, 0);     
     }
     else{
       analogWrite(motorR2, rightOutput*0.45);
       analogWrite(motorL2, 0);
       analogWrite(motorR1, 0);
       analogWrite(motorL1, rightOutput*0.45);        
     }
     ledsPID();
     readPosition(bno, event, mpu, 'B');
    endTime = millis();
  }
  blinkingLEDS();
  alignCenterPID(bno, event, mpu);
}


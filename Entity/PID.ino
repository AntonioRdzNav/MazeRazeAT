// -90 to turn left
// 0 to go forward
// 90 to turn right
int calculateNewSetpoint(int angle) {
  int newSetpoint = Setpoint;
  newSetpoint += angle;
  if (newSetpoint < -180)
    newSetpoint += 360;
  if (newSetpoint > 180)
    newSetpoint -= 360;
  if(abs(newSetpoint)==180)
    newSetpoint=abs(newSetpoint);
  return newSetpoint;
}

void forwardPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu) {
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

//void alignCenterPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu) {
//  double startTime = millis();
//  double endTime = 0;
//  readPosition(bno, event, mpu, 'B');
//  while (endTime - startTime < 5000 || (rightOutput == 0 && leftOutput == 0)) {
//    Serial.print(rawInput);
//    Serial.print("\t");
//    Serial.print(Input);
//    Serial.print("\t");
//    Serial.print(fakeInput);
//    Serial.print("\t");
//    bno.getEvent(&event);
//    xBNOKalmanFilter();
//    Input = xBNOXe;
//    leftPID.Compute();  //Gets an output
//    rightPID.Compute();
//    Serial.print(leftOutput);
//    Serial.print("\t");
//    Serial.println(rightOutput);
//    if (rightOutput == 0) {
//      analogWrite(motorL2, leftOutput);
//      analogWrite(motorR2, 0);
//      analogWrite(motorR1, leftOutput);
//      analogWrite(motorL1, 0);
//    }
//    else {
//      analogWrite(motorR2, rightOutput);
//      analogWrite(motorL2, 0);
//      analogWrite(motorR1, 0);
//      analogWrite(motorL1, rightOutput);
//    }
//    ledsPID();
//    readPosition(bno, event, mpu, 'B');
//    endTime = millis();
//  }
//  blinkingLEDS();
//}

void spinPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu){
  leftPID.SetTunings(rightTurnKp, rightTurnKi, rightTurnKd);
  rightPID.SetTunings(leftTurnKp, leftTurnKi, leftTurnKd);
  delay(300);
  turnPID(bno, event, mpu, 3000);
  leftPID.SetTunings(rightAlignKp, rightAlignKi, rightAlignKd);
  rightPID.SetTunings(leftAlignKp, leftAlignKi, leftAlignKd);
  turnPID(bno, event, mpu, 2500);
  leftPID.SetTunings(rightConsKp, rightConsKi, rightConsKd);
  rightPID.SetTunings(leftConsKp, leftConsKi, leftConsKd);
}

void turnPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu, double time) {
  double startTime = millis();
  double endTime = 0;
  readPosition(bno, event, mpu, 'B');
  while (endTime - startTime < time || (rightOutput == 0 && leftOutput == 0)) {
    Serial.print(rawInput);
    Serial.print("\t");
    Serial.print(Input);
    Serial.print("\t");
    Serial.print(fakeInput);
    Serial.print("\t");
    bno.getEvent(&event);
    xBNOKalmanFilter();
    Input = xBNOXe;
    leftPID.Compute();  //Gets an output
    rightPID.Compute();
    Serial.print(leftOutput);
    Serial.print("\t");
    Serial.println(rightOutput);
    if(abs(Setpoint)==180){
      if (Input>0) {
      analogWrite(motorL2, leftOutput);
      analogWrite(motorR2, 0);
      analogWrite(motorR1, leftOutput);
      analogWrite(motorL1, 0);
      }
      else {
        analogWrite(motorR2, leftOutput);
        analogWrite(motorL2, 0);
        analogWrite(motorR1, 0);
        analogWrite(motorL1, leftOutput);
      }
    }
    else{
      if (rightOutput == 0) {
      analogWrite(motorL2, leftOutput);
      analogWrite(motorR2, 0);
      analogWrite(motorR1, leftOutput);
      analogWrite(motorL1, 0);
      }
      else {
        analogWrite(motorR2, rightOutput);
        analogWrite(motorL2, 0);
        analogWrite(motorR1, 0);
        analogWrite(motorL1, rightOutput);
      }
    }
    ledsPID();
    readPosition(bno, event, mpu, 'B');
    endTime = millis();
  }
  blinkingLEDS();
}


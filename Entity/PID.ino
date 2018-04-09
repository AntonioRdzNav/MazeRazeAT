// -90 to turn left
// 0 to go forward
// 90 to turn right
void calculateNewSetpoint(int angle) {
  calculateFakeSetpoint(angle);
  int newSetpoint = Setpoint;
  newSetpoint += angle;
  if (newSetpoint < -180)
    newSetpoint += 360;
  if (newSetpoint > 180)
    newSetpoint -= 360;
  Setpoint = newSetpoint;
}
void calculateFakeSetpoint(int angle) {
  int newFakeSetpoint = fakeSetpoint;
  newFakeSetpoint += angle;
  if (newFakeSetpoint < -180)
    newFakeSetpoint += 360;
  if (newFakeSetpoint > 180)
    newFakeSetpoint -= 360;
  fakeSetpoint = newFakeSetpoint;
}
void setFakeSetpoint(){
  fakeSetpoint = Input;
}

void backPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu) {
  leftPID.SetTunings(4, rightTurnKi, rightTurnKd);
  rightPID.SetTunings(4, leftTurnKi, leftTurnKd);
  double startTime = millis();
  double endTime = 0;
  readPosition(bno, event, mpu, 'B');
  while (endTime - startTime < 1100 || (rightOutput == 0 && leftOutput == 0)) {
    leftPID.Compute();  //Gets an output
    rightPID.Compute();
    if(abs(Setpoint)==180){
      if(Setpoint==180){
        if (Input>0) {
        analogWrite(motorL2, 0);
        analogWrite(motorR1, velGenDerBack + leftOutput);
        analogWrite(motorR2, 0);
        analogWrite(motorL1, velGenIzqBack);//slowGo(endTime - startTime)
        }
        else {
          analogWrite(motorR2, 0);
          analogWrite(motorL1, velGenIzqBack + leftOutput);//velGenIzq
          analogWrite(motorR1, velGenDerBack);
          analogWrite(motorL2, 0);
        }
      }
      else if(Setpoint==-180){
        if (Input>0) {
        analogWrite(motorL2, 0);
        analogWrite(motorR1, velGenDerBack + rightOutput);
        analogWrite(motorR2, 0);
        analogWrite(motorL1, velGenIzqBack);//velGenIzq
        }
        else {
          analogWrite(motorR2, 0);
          analogWrite(motorL1, velGenIzqBack + rightOutput);//velGenIzq
          analogWrite(motorR1, velGenDerBack);
          analogWrite(motorL2, 0);
        }
      }
    }
    else{
    analogWrite(motorR2, 0);
    analogWrite(motorL2, 0);
    analogWrite(motorR1, velGenDerBack + leftOutput);
    analogWrite(motorL1, velGenIzqBack + rightOutput);//velGenIzq
    }
    ledsPID();
    readPosition(bno, event, mpu, 'B');
    endTime = millis();
  } 
  leftPID.SetTunings(rightConsKp, rightConsKi, rightConsKd);
  rightPID.SetTunings(leftConsKp, leftConsKi, leftConsKd);
}

void forwardPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu) {
  readPosition(bno, event, mpu, 'B');
  leftPID.Compute();  //Gets an output
  rightPID.Compute();
  ledsPID();
  if(abs(Setpoint)==180){
    if(Setpoint==180){
      if (Input>0) {
      analogWrite(motorL2, velGenIzq + leftOutput);//slowGo()
      analogWrite(motorR1, 0);
      analogWrite(motorR2, velGenDer);
      analogWrite(motorL1, 0);
      }
      else {
        analogWrite(motorR2, velGenDer + leftOutput);
        analogWrite(motorL1, 0);
        analogWrite(motorR1, 0);
        analogWrite(motorL2, velGenIzq);//slowGo()
      }
    }
    else if(Setpoint==-180){
      if (Input>0) {
      analogWrite(motorL2, velGenIzq + rightOutput);//slowGo()
      analogWrite(motorR1, 0);
      analogWrite(motorR2, velGenDer);
      analogWrite(motorL1, 0);
      }
      else {
        analogWrite(motorR2, velGenDer + rightOutput);
        analogWrite(motorL1, 0);
        analogWrite(motorR1, 0);
        analogWrite(motorL2, velGenIzq);//slowGo()
      }
    }
  }
  else{
  analogWrite(motorR2, velGenDer + rightOutput);
  analogWrite(motorL2, velGenIzq + leftOutput);//slowGo()
  analogWrite(motorR1, 0);
  analogWrite(motorL1, 0);
  }
}

void spinPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu, int newAngle, bool isDeadEnd){
  stop(true);
  bool canBack=false;
  lastSetpoint = Setpoint;
  calculateNewSetpoint(newAngle);
  Serial.println(fakeSetpoint);
  leftPID.SetTunings(rightTurnKp, rightTurnKi, rightTurnKd);
  rightPID.SetTunings(leftTurnKp, leftTurnKi, leftTurnKd);
  filtrateDistances(ultraFront, ultraRight, ultraLeft); 
  if(newAngle>0){
    if(ultraLeft.side)   
      canBack=true;
  }
  else{
    if(ultraRight.side)
      canBack=true;
  }
  turnPID(bno, event, mpu, 1600);//3000
  if(abs(fakeSetpoint)-abs(Input) > 50){
    leftPID.SetTunings(rightTurnKp, rightAlignKi, rightAlignKd);
    rightPID.SetTunings(leftTurnKp, leftAlignKi, leftAlignKd);
  }
  else if(abs(fakeSetpoint)-abs(Input)<=50 && abs(fakeSetpoint)-abs(Input)>30){
    leftPID.SetTunings(2.5, rightAlignKi, rightAlignKd);
    rightPID.SetTunings(2.5, leftAlignKi, leftAlignKd);
  }
  else if(abs(fakeSetpoint)-abs(Input) < 30){
    leftPID.SetTunings(rightAlignKp, rightAlignKi, rightAlignKd);
    rightPID.SetTunings(leftAlignKp, leftAlignKi, leftAlignKd);
  }
  turnPID(bno, event, mpu, 500);//2500
  leftPID.SetTunings(rightConsKp, rightConsKi, rightConsKd);
  rightPID.SetTunings(leftConsKp, leftConsKi, leftConsKd);
  if(canBack && !isDeadEnd){
     backPID(bno, event, mpu);      
     stop(false);
     setFakeSetpoint();   
     oneStep(ultraFront, ultraRight, ultraLeft, backStepDistance);
  }
  else
    oneStep(ultraFront, ultraRight, ultraLeft, stepDistance);
}

void turnPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu, double time) {
  double startTime = millis();
  double endTime = 0;
  double tempOutput;
  readPosition(bno, event, mpu, 'B');
  while(endTime - startTime < time) {
    filtrateDistances(ultraFront, ultraRight, ultraLeft);
    bno.getEvent(&event);
    leftPID.Compute();  //Gets an output
    rightPID.Compute();
    (leftOutput==0)? tempOutput=rightOutput: tempOutput=leftOutput;
    turnOnLeds();
    if(Setpoint==180u){
      if (Input>0) {
        analogWrite(motorL2, tempOutput);
        analogWrite(motorR2, 0);
        analogWrite(motorR1, tempOutput);
        analogWrite(motorL1, 0);
      }
      else {
        analogWrite(motorR2, tempOutput);
        analogWrite(motorL2, 0);
        analogWrite(motorR1, 0);
        analogWrite(motorL1, tempOutput);
      }
    }
    else if(Setpoint==-180){
      if (Input<0) {
      analogWrite(motorL2, 0);
      analogWrite(motorR2, tempOutput);
      analogWrite(motorR1, 0);
      analogWrite(motorL1, tempOutput);
      }
      else {
        analogWrite(motorR2, 0);
        analogWrite(motorL2, tempOutput);
        analogWrite(motorR1, tempOutput);
        analogWrite(motorL1, 0);
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
}


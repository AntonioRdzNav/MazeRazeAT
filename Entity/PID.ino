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
void setNewFakeSetpoint(){
  fakeSetpoint = Input;
}

void edoTensei(){
  fakeSetpoint = Setpoint;
}

void regulateOutputsPID(){
  if(leftOutput > maxTurnVel)
    leftOutput=maxTurnVel;
  if(rightOutput > maxTurnVel)
    rightOutput=maxTurnVel;
}

void backPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu, int time) {
  leftPID.SetTunings(rightConsKp-1.2, rightTurnKi, rightTurnKd);
  rightPID.SetTunings(leftConsKp-1.2, leftTurnKi, leftTurnKd);
  double startTime = millis();
  double endTime = 0;
//  bool limitDer=false, limitIzq=false;
  setFakeSetpoint=false;
  readPosition(bno, event, mpu, 'B');
  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack); 
  while (endTime - startTime < time)/* && (!limit1 || !limit2))*/ {  
    leftPID.Compute();  //Gets an output
    rightPID.Compute();
    regulateOutputsPID();
    digitalWrite(motorR1, HIGH);
    digitalWrite(motorR2, LOW);
    digitalWrite(motorL1, HIGH);
    digitalWrite(motorL2, LOW);     
    if(abs(Setpoint)==180){
      if(Setpoint==180){
        if (Input>0) {
          analogWrite(motorD_PWM, velGenDerBack + leftOutput);
          analogWrite(motorL_PWM, velGenIzqBack);          
        }
        else { 
          analogWrite(motorD_PWM, velGenDerBack);
          analogWrite(motorL_PWM, velGenIzqBack + leftOutput);          
        }
      }
      else if(Setpoint==-180){
        if (Input>0) {
          analogWrite(motorD_PWM, velGenDerBack + rightOutput);
          analogWrite(motorL_PWM, velGenIzqBack);           
        }
        else {
          analogWrite(motorD_PWM, velGenDerBack);
          analogWrite(motorL_PWM, velGenIzqBack + rightOutput);      
        }
      }
    }
    else{
      analogWrite(motorD_PWM, velGenDerBack + leftOutput);
      analogWrite(motorL_PWM, velGenIzqBack + rightOutput);      
    }
    ledsPID();
    readPosition(bno, event, mpu, 'B');
    filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack); 
//    limitDer = analogRead(limitSwitchDer);
//    limitIzq = analogRead(limitSwitchIzq);
//    if(limitDer && limitIzq)
    setFakeSetpoint=true;
//    checkColor();
    endTime = millis();
  }  
  leftPID.SetTunings(rightConsKp, rightConsKi, rightConsKd);
  rightPID.SetTunings(leftConsKp, leftConsKi, leftConsKd);
  stop(false);
}

void forwardPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu) {
  readPosition(bno, event, mpu, 'B');     
  leftPID.Compute();  //Gets an output
  rightPID.Compute();
  ledsPID();
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, HIGH);
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH); 
  //Aqui empieza
  if(abs(Setpoint) == 180){
//    int tempPID = fakeInput < fakeSetpoint ? leftOutput : rightOutput;
    if(fakeSetpoint > 0){
      if(Input > 0 && Input < fakeSetpoint){
        analogWrite(motorD_PWM, velGenDer);
        analogWrite(motorL_PWM, velGenIzq + leftOutput + rightOutput); 
        
      }
      else{
        analogWrite(motorD_PWM, velGenDer + (leftOutput + rightOutput)*(3.5)/**tempPID*/);
        analogWrite(motorL_PWM, velGenIzq); 
      }
    }
    else{
      if(Input < 0 && Input > fakeSetpoint){
        analogWrite(motorD_PWM, velGenDer + (leftOutput + rightOutput)*(3.5)/**tempPID*/);
        analogWrite(motorL_PWM, velGenIzq); 
      }
      else{
        analogWrite(motorD_PWM, velGenDer);
        analogWrite(motorL_PWM, velGenIzq + leftOutput + rightOutput); 
      }
    }
  }
//  if(abs(Setpoint)==180){
//    if(Setpoint==180){
//      if (Input>0) {
//        analogWrite(motorD_PWM, velGenDer);
//        analogWrite(motorL_PWM, velGenIzq + leftOutput);  
//        digitalWrite(ledGreen, HIGH); digitalWrite(ledRed, LOW);
//      }
//      else {
//        analogWrite(motorD_PWM, velGenDer + leftOutput);
//        analogWrite(motorL_PWM, velGenIzq);  
//        digitalWrite(ledRed, HIGH); digitalWrite(ledGreen, LOW);
//      }
//    }
//    else if(Setpoint==-180){
//      if (Input>0) {
//        analogWrite(motorD_PWM, velGenDer);
//        analogWrite(motorL_PWM, velGenIzq + rightOutput); 
//        digitalWrite(ledGreen, HIGH); digitalWrite(ledRed, LOW);        
//      }
//      else {
//        analogWrite(motorD_PWM, velGenDer + rightOutput);
//        analogWrite(motorL_PWM, velGenIzq);    
//        digitalWrite(ledRed, HIGH); digitalWrite(ledGreen, LOW);      
//      }
//    }
//  }
  else{
    analogWrite(motorD_PWM, velGenDer + rightOutput);
    analogWrite(motorL_PWM, velGenIzq + leftOutput);  
    ledsPID();    
  }
//  checkColor();   
  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);   
}

void spinPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu, int newAngle, bool isDeadEnd){
  if(!LARC)
    stop(true);
  bool canBack=false;
  lastSetpoint = Setpoint;
  calculateNewSetpoint(newAngle);
  leftPID.SetTunings(rightTurnKp, rightTurnKi, rightTurnKd);
  rightPID.SetTunings(leftTurnKp, leftTurnKi, leftTurnKd);
  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack); 
  if(newAngle>0){
    if(ultraLeft.side)   
      canBack=true;
  }
  else{
    if(ultraRight.side)
      canBack=true;
  }
  if(LARC)
    canBack=false;
  turnPID(bno, event, mpu, 1800);//3000
  if(abs(fakeSetpoint)-abs(Input) > 50){
    leftPID.SetTunings(rightTurnKp, rightAlignKi, rightAlignKd);
    rightPID.SetTunings(leftTurnKp, leftAlignKi, leftAlignKd);
  }
  else if(abs(fakeSetpoint)-abs(Input)<=50 && abs(fakeSetpoint)-abs(Input)>30){
    double middleLeft = (leftTurnKp+leftAlignKp) / 2.0;
    double middleRight = (rightTurnKp+rightAlignKp) / 2.0;
    leftPID.SetTunings(middleRight, rightAlignKi, rightAlignKd);
    rightPID.SetTunings(middleLeft  , leftAlignKi, leftAlignKd);
  }
  else if(abs(fakeSetpoint)-abs(Input) < 30){
    leftPID.SetTunings(rightAlignKp, rightAlignKi, rightAlignKd);
    rightPID.SetTunings(leftAlignKp, leftAlignKi, leftAlignKd);
  }
  turnPID(bno, event, mpu, 900);//2500
  leftPID.SetTunings(rightConsKp, rightConsKi, rightConsKd);
  rightPID.SetTunings(leftConsKp, leftConsKi, leftConsKd);
  if(canBack && !isDeadEnd){
     backPID(bno, event, mpu, 1300);     
     stop(false);
//     if(setFakeSetpoint)
        setNewFakeSetpoint();   
     oneStepMillis(true);
  }
  else
    oneStepMillis(false);
}

void turnPID(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu, double time) {
  double startTime = millis();
  double endTime = 0;
  double tempOutput;
  readPosition(bno, event, mpu, 'B');
  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack); 
  while(endTime - startTime < time) {
    bno.getEvent(&event);
    leftPID.Compute();  //Gets an output
    rightPID.Compute();
    regulateOutputsPID();
    (leftOutput==0)? tempOutput=rightOutput: tempOutput=leftOutput;
    if(Setpoint==180){
      if (Input>0) {
        digitalWrite(motorR1, HIGH);
        digitalWrite(motorR2, LOW);
        digitalWrite(motorL1, LOW);
        digitalWrite(motorL2, HIGH);   
        analogWrite(motorD_PWM, tempOutput);
        analogWrite(motorL_PWM, tempOutput);  
      }
      else {
        digitalWrite(motorR1, LOW);
        digitalWrite(motorR2, HIGH);
        digitalWrite(motorL1, HIGH);
        digitalWrite(motorL2, LOW);   
        analogWrite(motorD_PWM, tempOutput);
        analogWrite(motorL_PWM, tempOutput);          
      }
    }
    else if(Setpoint==-180){ //MOVED
      if (Input<0) {
        digitalWrite(motorR1, LOW);
        digitalWrite(motorR2, HIGH);
        digitalWrite(motorL1, HIGH);
        digitalWrite(motorL2, LOW);   
        analogWrite(motorD_PWM, tempOutput);
        analogWrite(motorL_PWM, tempOutput);        
      }
      else {
        digitalWrite(motorR1, HIGH);
        digitalWrite(motorR2, LOW);
        digitalWrite(motorL1, LOW);
        digitalWrite(motorL2, HIGH);   
        analogWrite(motorD_PWM, tempOutput);
        analogWrite(motorL_PWM, tempOutput);        
      }
    }
    else{
      if (rightOutput == 0) {
        digitalWrite(motorR1, HIGH);
        digitalWrite(motorR2, LOW);
        digitalWrite(motorL1, LOW);
        digitalWrite(motorL2, HIGH);   
        analogWrite(motorD_PWM, leftOutput);
        analogWrite(motorL_PWM, leftOutput);         
      }
      else {
        digitalWrite(motorR1, LOW);
        digitalWrite(motorR2, HIGH);
        digitalWrite(motorL1, HIGH);
        digitalWrite(motorL2, LOW);   
        analogWrite(motorD_PWM, rightOutput);
        analogWrite(motorL_PWM, rightOutput);            
      }
    }
    ledsPID();
    readPosition(bno, event, mpu, 'B');
    filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);    
    endTime = millis();
  }
}


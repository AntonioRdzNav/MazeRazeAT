void readPosition(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu, char selectGyro){
  if(selectGyro=='B')
    readBNO(bno, event);
  else if(selectGyro=='I')
    readIMU(mpu);
}

void readBNO(Adafruit_BNO055 &bno, sensors_event_t &event){
  bno.getEvent(&event);
  rawInput=event.orientation.x;
  Input = rawInput;
  if(Input>180) 
    Input = Input - 360;
  fakeInput = Input;
  if((Setpoint==90&&abs(lastSetpoint)==180) || (Setpoint==180))
    fakeInput = abs(Input);
  if((Setpoint==-90 && abs(lastSetpoint)==180) || (Setpoint==-180))
    fakeInput = -1*abs(Input);
}

//void printBNO(sensors_event_t event){
void printBNO(){
  Serial.print("X: ");
  Serial.print(event.orientation.x, 5);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 5);
  Serial.print("\t\tZ: ");
  Serial.print(event.orientation.z, 5);
  Serial.println("");
}

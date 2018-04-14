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
  if((abs(Setpoint)==90&&abs(lastSetpoint)==180) || abs(Setpoint)==180)
    fakeInput = fakeSetpoint > 0 ? abs(Input) : -1*abs(Input);
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

void getCalStat(){
  // Dave's Mod - Move sensor to calibrate, when status shows calibration, read values
byte cal = bno.getCalib();
  byte calSys = (0xC0 & cal) >> 6;
  byte calGyro = (0x30 & cal) >> 4;
  byte calAccel = (0x0C & cal) >> 2;
  byte calMag = (0x03 & cal) >> 0;
  
  Serial.println(cal);
  Serial.print("System calibration status "); Serial.println(calSys);
  Serial.print("Gyro   calibration status "); Serial.println(calGyro);
  Serial.print("Accel  calibration status "); Serial.println(calAccel);
  Serial.print("Mag    calibration status "); Serial.println(calMag);
  
  delay(1000);
  if (cal==255){
    getCal();
  }
}

void getCal(){
  // Dave's Mod - Reads Calibration Data when sensors are calibrated
  byte calData;
  bno.setMode( bno.OPERATION_MODE_CONFIG );    // Put into CONFIG_Mode
  
  calData = bno.getCalvalARL();
  Serial.println(calData);
  
  calData = bno.getCalvalARM();
  Serial.println(calData);
  
  calData = bno.getCalvalMRL();
  Serial.println(calData);
  
  calData = bno.getCalvalMRM();
  Serial.println(calData);
  
  calData = bno.getCalvalAOXL();
  Serial.println(calData);
  
  calData = bno.getCalvalAOXM();
  Serial.println(calData);
  
  calData = bno.getCalvalAOYL();
  Serial.println(calData);
  
  calData = bno.getCalvalAOYM();
  Serial.println(calData);
  
  calData = bno.getCalvalAOZL();
  Serial.println(calData);
 
  calData = bno.getCalvalAOZM();
  Serial.println(calData);
  
  calData = bno.getCalvalMOXL();
  Serial.println(calData);
  
  calData = bno.getCalvalMOXM();
  Serial.println(calData);
  
  calData = bno.getCalvalMOYL();
  Serial.println(calData);
 
  calData = bno.getCalvalMOYM();
  Serial.println(calData);
 
  calData = bno.getCalvalMOZL();
  Serial.println(calData);
  
  calData = bno.getCalvalMOZM();
  Serial.println(calData);
  
  calData = bno.getCalvalGOXL();
  Serial.println(calData);
  
  calData = bno.getCalvalGOXM();
  Serial.println(calData);
  
  calData = bno.getCalvalGOYL();
  Serial.println(calData);
  
  calData = bno.getCalvalGOYM();
  Serial.println(calData);
  
  calData = bno.getCalvalGOZL();
  Serial.println(calData);
  
  calData = bno.getCalvalGOZM();
  Serial.println(calData);
  
  while(1){                              // Stop
    delay(1000);
  }
}

void setCal(){
  // DAVES MOD - Writes calibration data to sensor//
  byte calData;
  bno.setMode( bno.OPERATION_MODE_CONFIG );    // Put into CONFIG_Mode
  delay(25);
  
  calData = bno.setCalvalARL(232);
  
  calData = bno.setCalvalARM(3);
  
  calData = bno.setCalvalMRL(87);
  
  calData = bno.setCalvalMRM(3);
  
  calData = bno.setCalvalAOXL(231);
  
  calData = bno.setCalvalAOXM(255);
  
  calData = bno.setCalvalAOYL(253);
  
  calData = bno.setCalvalAOYM(255);
  
  calData = bno.setCalvalAOZL(5);
 
  calData = bno.setCalvalAOZM(0);
  
  calData = bno.setCalvalMOXL(14);
  
  calData = bno.setCalvalMOXM(1);
  
  calData = bno.setCalvalMOYL(127);
 
  calData = bno.setCalvalMOYM(0);
 
  calData = bno.setCalvalMOZL(173);
  
  calData = bno.setCalvalMOZM(1);
  
  calData = bno.setCalvalGOXL(155);
  
  calData = bno.setCalvalGOXM(255);
  
  calData = bno.setCalvalGOYL(255);
  
  calData = bno.setCalvalGOYM(255);
  
  calData = bno.setCalvalGOZL(254);
  
  calData = bno.setCalvalGOZM(255);
  
  bno.setMode( bno.OPERATION_MODE_NDOF );    // Put into NDOF Mode
  delay(25);
}

void readIMU(MPU6050 &mpu){
  rawGyro = mpu.readRawGyro();
  normGyro = mpu.readNormalizeGyro();
  Input = normGyro.XAxis;
}

void printIMU(){
  Serial.print(" Xraw = ");
  Serial.print(rawGyro.XAxis);
  Serial.print(" Yraw = ");
  Serial.print(rawGyro.YAxis);
  Serial.print(" Zraw = ");
  Serial.print(rawGyro.ZAxis);
 
  Serial.print(" Xnorm = ");
  Serial.print(normGyro.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(normGyro.YAxis);
  Serial.print(" Znorm = ");
  Serial.println(normGyro.ZAxis);
}


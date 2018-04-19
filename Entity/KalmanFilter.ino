/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////ULTRASONIC/////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void ultraKalmanFilter(UltraKalman &ultra){
	ultra.Pc = ultra.P + ultra.varProcess;
	ultra.G = ultra.Pc/(ultra.Pc + ultra.varSensor); 	//Kalman gain
	ultra.P = (1-ultra.G)*ultra.Pc;
	ultra.Xp = ultra.Xe;
	ultra.Zp = ultra.Xp;
	ultra.Xe = ultra.G*(ultra.distance-ultra.Zp)+ultra.Xp;	//Estimates new filtered input
}
void ultra_RawKalman(UltraKalman &ultra, NewPing ping){
  rawUltrasonic(ultra, ping);
  Serial.print(ultra.distance);
  Serial.print(",");
  ultraKalmanFilter(ultra);
  Serial.println(ultra.Xe);
}
void filtrateDistances(UltraKalman &ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft, UltraKalman &ultraBack){
  calculateRawDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
  ultraKalmanFilter(ultraFront);
  ultraKalmanFilter(ultraRight);
  ultraKalmanFilter(ultraLeft);
  ultraKalmanFilter(ultraBack);
//  if(ultraFront.Xe <= 15 && ultraFront.Xe!=0) ultraNegativoSide = true;
//  else ultraNegativoSide = false;
  if(ultraBack.Xe <= 25 && ultraBack.Xe!=0) ultraBack.side = true;
  else ultraBack.side = false;
  if(ultraFront.Xe <= 25 && ultraFront.Xe!=0) ultraFront.side = true;
  else ultraFront.side = false;
  if(ultraRight.Xe <= 25 && ultraRight.Xe != 0) ultraRight.side = true;
  else ultraRight.side = false;
  if(ultraLeft.Xe <= 25 && ultraLeft.Xe != 0) ultraLeft.side = true;
  else ultraLeft.side = false;
}

/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////BNO////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void xBNOKalmanFilter(){
  xBNOPc = xBNOP + xBNOVarProcess;
  xBNOG = xBNOPc/(xBNOPc + xBNOVarSensor);  //Kalman gain
  xBNOP = (1-xBNOG)*xBNOPc;
  xBNOXp = xBNOXe;
  xBNOZp = xBNOXp;
  xBNOXe = xBNOG*(Input-xBNOZp)+xBNOXp;  //Estimates new filtered input
}
void xBNO_RawKalman(Adafruit_BNO055 &bno, sensors_event_t &event){
  readPosition(bno, event, mpu, 'B');
  Serial.print(Input);
  Serial.print(",");
  xBNOKalmanFilter();
  Serial.println(xBNOXe);
}

/////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////IMU////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void xIMUKalmanFilter(){
  xIMUPc = xIMUP + xIMUVarProcess;
  xIMUG = xIMUPc/(xIMUPc + xIMUVarSensor);  //Kalman gain
  xIMUP = (1-xIMUG)*xIMUPc;
  xIMUXp = xIMUXe;
  xIMUZp = xIMUXp;
  xIMUXe = xIMUG*(Input-xIMUZp)+xIMUXp;  //Estimates new filtered input
}
void xIMU_RawKalman(Adafruit_BNO055 &bno, sensors_event_t &event, MPU6050 &mpu){
  readPosition(bno, event, mpu, 'I');
  Serial.print(Input);
  Serial.print(",");
  xIMUKalmanFilter();
  Serial.println(xIMUXe);
}

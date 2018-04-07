void rightPriotity(UltraKalman &ultraRight, UltraKalman &ultraLeft, UltraKalman &ultraFront){  
  double startTime;
  double endTime;
  double startDistance;
  filtrateDistances(ultraFront, ultraRight, ultraLeft);
  readPosition(bno, event, mpu, 'B');
  if(!ultraRight.side){ 
   if(ultraLeft.side){
      spinPID(bno, event, mpu, 90);
      backPID(bno, event, mpu);
      stop(false);
   }
   else
      spinPID(bno, event, mpu, 90);
    startTime = millis();
    endTime = 0;
    startDistance = ultraFront.distance;
    Serial.print(startDistance);
    Serial.print("\t");
//  delay(2000);
    while(ultraFront.distance>startDistance-39){
      Serial.print(startDistance-39);
      Serial.print("\t\t\t");
      Serial.println(ultraFront.distance);
      forwardPID(bno, event, mpu);
      filtrateDistances(ultraFront, ultraRight, ultraLeft);
//      endTime=millis();
    } 
    stop(true);
  }
  else if (!ultraFront.side){
    startTime = millis();
    endTime = 0;
    startDistance = ultraFront.distance;
    Serial.print(startDistance);
    Serial.print("\t");
//  delay(2000);
    while(ultraFront.distance>startDistance-39){
      Serial.print(startDistance-39);
      Serial.print("\t\t\t");
      Serial.println(ultraFront.distance);
      forwardPID(bno, event, mpu);
      filtrateDistances(ultraFront, ultraRight, ultraLeft);
      endTime=millis();
    } 
    stop(true);
  }
  else if(!ultraLeft.side){
    if(ultraRight.side){
       spinPID(bno, event, mpu, -90);
       backPID(bno, event, mpu);
       stop(false);
    }
    else
      spinPID(bno, event, mpu, -90);
    startTime = millis();
    endTime = 0;
    startDistance = ultraFront.distance;
    Serial.print(startDistance);
    Serial.print("\t");
//  delay(2000);
    while(ultraFront.distance>startDistance-39){
      Serial.print(startDistance-39);
      Serial.print("\t\t\t");
      Serial.println(ultraFront.distance);
      forwardPID(bno, event, mpu);
      filtrateDistances(ultraFront, ultraRight, ultraLeft);
      endTime=millis();
    } 
    stop(true);      
  }
  else{
   spinPID(bno, event, mpu, 90);
   spinPID(bno, event, mpu, 90);
  }  
}

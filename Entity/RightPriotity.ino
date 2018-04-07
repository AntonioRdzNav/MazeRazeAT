void rightPriotity(UltraKalman &ultraRight, UltraKalman &ultraLeft, UltraKalman &ultraFront){  
  double startTime = millis();
  double endTime = 0;
  filtrateDistances(ultraRight, ultraLeft, ultraFront);
  readPosition(bno, event, mpu, 'B');
  if(!ultraRight.side){ 
   if(ultraLeft.side){
      spinPID(bno, event, mpu, 90);
      backPID(bno, event, mpu);
      stop(false);
   }
   else
      spinPID(bno, event, mpu, 90);
  }
  else if (!ultraFront.side){
    while(endTime - startTime < 1000){
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
  }
  else{
   spinPID(bno, event, mpu, 90);
   spinPID(bno, event, mpu, 90);
  }  
}

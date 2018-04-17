void oneStep(UltraKalman ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft, double distanceCM){
    double startDistance = ultraFront.distance;
    if(startDistance-distanceCM < 5){
      while(!ultraNegativoSide){
        forwardPID(bno, event, mpu);
        filtrateDistances(ultraFront, ultraRight, ultraLeft);
      } 
    }
    else{
      while(ultraFront.distance>startDistance-distanceCM){
        forwardPID(bno, event, mpu);
        filtrateDistances(ultraFront, ultraRight, ultraLeft);
      } 
    }
}

void rightPriotity(UltraKalman &ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft){ 
  filtrateDistances(ultraFront, ultraRight, ultraLeft);
  readPosition(bno, event, mpu, 'B');
  if(!ultraRight.side){ 
    spinPID(bno, event, mpu, 90, false);
  }
  else if (!ultraFront.side){
    oneStep(ultraFront, ultraRight, ultraLeft, stepDistance);
  }
  else if(!ultraLeft.side){
    spinPID(bno, event, mpu, -90, false);  
  }
  else{
    filtrateDistances(ultraFront, ultraRight, ultraLeft);
    if(ultraRight.distance > ultraLeft.distance)
      spinPID(bno, event, mpu, 180, false);
    else 
      spinPID(bno, event, mpu, -180, false);
  }  
}

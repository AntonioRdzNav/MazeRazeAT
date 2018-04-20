void oneStep(UltraKalman ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft, bool comeFromBack){
    filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
    double distanceCM;
    (comeFromBack)? (distanceCM = backStepDistance) : (distanceCM = stepDistance); 
    double startDistance = ultraFront.distance;
    if(startDistance-distanceCM < 5){
      while(ultraFront.distance > 12){
        forwardPID(bno, event, mpu);
        filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack); // PASO HASTA PARED (usando ultraFront)
      } 
    }
    else{
      if(comeFromBack){
        while(ultraBack.distance<distanceCM){
          forwardPID(bno, event, mpu);
          filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
        } 
      }
      else if(ultraFront.distance < (ultraBack.distance+distanceCM)){
        while(ultraFront.distance>=startDistance-distanceCM){
          forwardPID(bno, event, mpu);
          filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
        }  // PASO NORMAL (usando ultraFront)
      }
      else{
        filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
        startDistance = ultraBack.distance;
        while(ultraBack.distance<startDistance+distanceCM){
          forwardPID(bno, event, mpu);
          filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
        } 
      }
    }
}

void oneStepMillis(bool comeFromBack){
  double time;
  int tempColor;
  switchColor = false;
  (comeFromBack)? (time = 1000): (time = 880);
  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
  double startTime = millis();
  double endTime = 0;
  while ((endTime - startTime < time) && (ultraFront.distance > 8)){  
    forwardPID(bno, event, mpu);
    endTime = millis();
    updateColors(currentColor());
    if(!switchColor){
      tempColor = currentColor();
      switchColor = (tempColor!=-1) && (tempColor!=4) && (tempColor!=3);
    }
    if(switchColor && currentColor()==4){
      stopColor(tempColor);
      time+=2000;
      switchColor = false; 
    }
    filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
  } 
}

void exitRoutine(){
  if(Setpoint == 0)
    oneStepMillis(false);
  else if(Setpoint == 90)
    spinPID(bno, event, mpu, -90, false);
  else if(Setpoint == -90)
    spinPID(bno, event, mpu, 90, false);
}

void rightPriotity(UltraKalman &ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft){ 
  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
  if(colorRedDetected && colorGreenDetected && colorBlackDetected){
    exitRoutine();  
    while(1){
      stop(false);
    }
  }
  if((fakeSetpoint*Setpoint)<0 || (abs(fakeSetpoint)-abs(Setpoint))>90)
    edoTensei();
  readPosition(bno, event, mpu, 'B');
  if(!ultraRight.side){ 
    spinPID(bno, event, mpu, 90, false);
  }
  else if (!ultraFront.side){
    oneStepMillis(firstBack);
    firstBack=false;
  }
  else if(!ultraLeft.side){
    spinPID(bno, event, mpu, -90, false);  
  }
  else{
    filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
    if(ultraRight.distance > ultraLeft.distance)
      spinPID(bno, event, mpu, 180, false);
    else{
      oneStepMillis(firstBack);
      spinPID(bno, event, mpu, -180, false);
    }
  }  
}

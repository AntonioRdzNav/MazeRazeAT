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
  int limitDer=0, limitIzq=0;  
  switchColor = false;
  (comeFromBack)? (time = timeStepBack): (time = timeStep);
  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
  double startTime = millis();
  double endTime = 0;
  while ((endTime - startTime < time) && (ultraFront.distance > 8)){ 
    limitDer = analogRead(limitSwitchDer);
    limitIzq = analogRead(limitSwitchIzq);
    if(limitDer > 500){
      backPID(bno, event, mpu, 400);
      spinPID(bno, event, mpu, -90, false);
      break;
    }
    if(limitIzq > 500){
      backPID(bno, event, mpu, 400);
      spinPID(bno, event, mpu, 90, false);   
      break;    
    }
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
    forwardPID(bno, event, mpu);
    endTime = millis();    
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
//    exitRoutine();  
    while(1){
      stop(false);
    }
  }
  if((fakeSetpoint*Setpoint)<0 || (abs(fakeSetpoint)-abs(Setpoint))>50)
    edoTensei();
  readPosition(bno, event, mpu, 'B');
  if(!ultraRight.side){
    if(turnsCounter < 4){ 
      spinPID(bno, event, mpu, 90, false);
      turnsCounter++;
    }
    else{
      if(!ultraLeft.side){
        spinPID(bno, event, mpu, -90, false);  
        turnsCounter = 0;
        firstBack=false;
      }
      else if(!ultraFront.side){
        oneStepMillis(firstBack);
        firstBack=false;
        turnsCounter = 0;
      }
    }
  }
  else if (!ultraFront.side){
    oneStepMillis(firstBack);
    firstBack=false;
    turnsCounter = 0;
  }
  else if(!ultraLeft.side){
    spinPID(bno, event, mpu, -90, false);  
    turnsCounter = 0;
  }
  else{
    filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
    turnsCounter = 0;
    if(ultraRight.distance > ultraLeft.distance)
      spinPID(bno, event, mpu, 180, false);
    else{
      oneStepMillis(firstBack);
      spinPID(bno, event, mpu, -180, false);
    }
  }  
}

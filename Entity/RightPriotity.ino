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
          analogWrite(ledBlue, HIGH); // PASO BACK (usando ultraBack)
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
//          analogWrite(ledBlue, HIGH); // PASO NORMAL (usando ultraBack)
        } 
      }
    }
//    analogWrite(ledBlue, LOW);
}

void oneStepMillis(bool comeFromBack){
  analogWrite(ledBlue, HIGH);
  double time;
  (comeFromBack)? (time = 1000): (time = 880);
  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
  double startTime = millis();
  double endTime = 0;
  while ((endTime - startTime < time) && (ultraFront.distance > 8)){  
    forwardPID(bno, event, mpu);
    endTime = millis();
    filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
  }  
  analogWrite(ledBlue, LOW);
}

void saveDoorBits(){
  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack); 
  if(abs(Setpoint) == 180){
    bits[0] = ultraRight.side;
    bits[1] = false;
    bits[2] = ultraLeft.side;
    bits[3] = ultraFront.side;
  }
  else if(Setpoint == 90){
    bits[0] = false;
    bits[1] = ultraLeft.side;
    bits[2] = ultraFront.side;
    bits[3] = ultraRight.side;
  }
  else if(Setpoint == -90){
    bits[0] = ultraFront.side;
    bits[1] = ultraLeft.side;
    bits[2] = ultraFront.side;
    bits[3] = ultraRight.side;
  }
}

void rightPriotity(UltraKalman &ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft){ 
  //Empieza
    colorDecision();
  //Termina
  filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
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

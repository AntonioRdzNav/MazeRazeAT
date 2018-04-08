void oneStep(UltraKalman ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft, double distanceCM){
    double startDistance = ultraFront.distance;
//    Serial.print(startDistance);
//    Serial.print("\t");
    if(startDistance-distanceCM < 5)
      while(!ultraFront.side){
//        Serial.println(ultraFront.distance);
        forwardPID(bno, event, mpu);
        filtrateDistances(ultraFront, ultraRight, ultraLeft);
      }  
    else    
      while(ultraFront.distance>startDistance-distanceCM){
//        Serial.print(startDistance-distanceCM);
//        Serial.print("\t\t\t");
//        Serial.println(ultraFront.distance);
        forwardPID(bno, event, mpu);
        filtrateDistances(ultraFront, ultraRight, ultraLeft);
      } 
    stop(true);
}

void rightPriotity(UltraKalman &ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft){ 
  filtrateDistances(ultraFront, ultraRight, ultraLeft);
  readPosition(bno, event, mpu, 'B');
  if(!ultraRight.side){ 
   if(ultraLeft.side){
      spinPID(bno, event, mpu, 90);
      backPID(bno, event, mpu);
      stop(false);
      setFakeSetpoint();
      Serial.println(Setpoint);      
      oneStep(ultraFront, ultraRight, ultraLeft, 34);
   }
   else{
      spinPID(bno, event, mpu, 90);
      oneStep(ultraFront, ultraRight, ultraLeft, 24);
   }
  }
  else if (!ultraFront.side){
    oneStep(ultraFront, ultraRight, ultraLeft, 34);
  }
  else if(!ultraLeft.side){
    if(ultraRight.side){
       spinPID(bno, event, mpu, -90);
       backPID(bno, event, mpu);      
       stop(false);
       setFakeSetpoint();
       Serial.println(Setpoint);       
       oneStep(ultraFront, ultraRight, ultraLeft, 34);
    }
    else{
      spinPID(bno, event, mpu, -90);
      oneStep(ultraFront, ultraRight, ultraLeft, 24);
    }   
  }
  else{
   spinPID(bno, event, mpu, 90);
   spinPID(bno, event, mpu, 90);
   backPID(bno, event, mpu);
   stop(false);
   setFakeSetpoint();
   Serial.println(Setpoint);
  }  
}

void rightPriotity(UltraKalman &ultraRight, UltraKalman &ultraLeft, UltraKalman &ultraFront){  
  filtrateDistances(ultraRight, ultraLeft, ultraFront);
  readPosition(bno, event, mpu, 'B');
  if(!ultraRight.side){ 
   lastSetpoint = Setpoint;
   Setpoint = calculateNewSetpoint(90);
   spinPID(bno, event, mpu);
  }
  else if (!frente){
    forwardPID(bno, event, mpu);
    ledsPID();
  }
  else if(!izquierda){
   lastSetpoint = Setpoint;
   Setpoint = calculateNewSetpoint(-90);
   spinPID(bno, event, mpu);
  }
  else{
   lastSetpoint = Setpoint;
   Setpoint = calculateNewSetpoint(90);
   spinPID(bno, event, mpu);
   lastSetpoint = Setpoint;
   Setpoint = calculateNewSetpoint(90);
   spinPID(bno, event, mpu);
  }  
}

void firstChallenge(){
  ////////////////////////////////////////////////////////////////////////////
  range = 23;
  velGenDer = 60;
  velGenIzq = 60;
  velGenDerBack = 60;
  velGenIzqBack = 60;  
  leftAlignKp=3.4, leftAlignKi=0, leftAlignKd=0;
  leftTurnKp=1.5, leftTurnKi=0, leftTurnKd=0;
  leftConsKp=3.4, leftConsKi=0, leftConsKd=0;
  leftGenKp=leftConsKp, leftGenKi=leftConsKi, leftGenKd=leftConsKd;
  leftError=0;
  rightAlignKp=3.4, rightAlignKi=0, rightAlignKd=0;
  rightTurnKp= 1.5, rightTurnKi=0, rightTurnKd=0;
  rightConsKp=3.5, rightConsKi=0, rightConsKd=0;
  rightGenKp=rightConsKp, rightGenKi=rightConsKi, rightGenKd=rightConsKd;    
  /////////////////////////////////////////////////////////////////////////////
  calibrarColores(1);
  delay(5000);
  while(1){
    while(currentColor() != 1){
      forwardPID(bno, event, mpu);
    }
    stop(true);
//    setNewFakeSetpoint();     
    backPID(bno, event, mpu, 220);
    spinPID(bno, event, mpu, -90, true);  
  }
}


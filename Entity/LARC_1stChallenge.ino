void firstChallenge(){
  velGenDer = 65;
  velGenIzq = 65-15;
  calibrarColores(1);
  delay(2000);
  while(1){
    while(currentColor() != 1){
      forwardPID(bno, event, mpu);
    }
    stop(true);
    backPID(bno, event, mpu, 300);
    spinPID(bno, event, mpu, -90, true);  
  }
}


double slowGo(double time){
  return 65*(1-exp(-0.01*time/5));
}

void stop(bool forward){    
  if(forward){
    analogWrite(motorR1, 235);
    analogWrite(motorR2, 0);
    analogWrite(motorL1, 235);
    analogWrite(motorL2, 0); 
    delay(140);   
  }                                                                                  
  analogWrite(motorR1, 0);
  analogWrite(motorR2, 0);
  analogWrite(motorL1, 0);
  analogWrite(motorL2, 0);
}

void go() {
  analogWrite(motorR1, 0);
  analogWrite(motorR2, 80);
  analogWrite(motorL1, 0);
  analogWrite(motorL2, 80);
}

void back() {
  analogWrite(motorR1, 45);
  analogWrite(motorR2, 0);
  analogWrite(motorL1, 45);
  analogWrite(motorL2, 0);
}

void turnRight(){                                                                              
    analogWrite(motorR1, 65);
    analogWrite(motorR2, 0);
    analogWrite(motorL1, 0);
    analogWrite(motorL2, 65);
    delay(200);
    stop(false);
}

void turnLeft(){                                                                              
    analogWrite(motorR1, 0);
    analogWrite(motorR2, 50);
    analogWrite(motorL1, 50);
    analogWrite(motorL2, 0);
    delay(200);
    stop(false);
}

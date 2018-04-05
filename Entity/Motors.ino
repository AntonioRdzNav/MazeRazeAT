double slowGo(double time){
  return 65*(1-exp(-0.01*time/5));
}

void stop(){    
  analogWrite(motorR1, 60);
  analogWrite(motorR2, 0);
  analogWrite(motorL1, 60);
  analogWrite(motorL2, 0); 
  delay(50);                                                                                     
  analogWrite(motorR1, 0);
  analogWrite(motorR2, 0);
  analogWrite(motorL1, 0);
  analogWrite(motorL2, 0);
}

void go() {
  analogWrite(motorR1, 0);
  analogWrite(motorR2, 60);
  analogWrite(motorL1, 0);
  analogWrite(motorL2, 60);
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
    stop();
}

void turnLeft(){                                                                              
    analogWrite(motorR1, 0);
    analogWrite(motorR2, 50);
    analogWrite(motorL1, 50);
    analogWrite(motorL2, 0);
    delay(200);
//    stop();
}

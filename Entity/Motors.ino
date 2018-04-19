double slowGo(double time){
  return 65*(1-exp(-0.01*time/5));
}

void stop(bool isSpin){   
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, LOW);
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, LOW);   
  if(isSpin){    
    digitalWrite(motorR1, HIGH);
    digitalWrite(motorR2, LOW);
    digitalWrite(motorL1, HIGH);
    digitalWrite(motorL2, LOW); 
    analogWrite(motorD_PWM, 235);
    analogWrite(motorL_PWM, 235);
    delay(70);   
  }                                                                  
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, LOW);
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, LOW);
//  delay(100); 
}      

void go() {
    digitalWrite(motorR1, LOW);
    digitalWrite(motorR2, HIGH);
    digitalWrite(motorL1, LOW);
    digitalWrite(motorL2, HIGH); 
    analogWrite(motorD_PWM, 100);
    analogWrite(motorL_PWM, 100);
}

void back() {
    digitalWrite(motorR1, HIGH);
    digitalWrite(motorR2, LOW);
    digitalWrite(motorL1, HIGH);
    digitalWrite(motorL2, LOW); 
    analogWrite(motorD_PWM, 110);
    analogWrite(motorL_PWM, 110);
  delay(2000);
}
//
//void turnRight(){                                                                              
//    analogWrite(motorR1, 10);
//    analogWrite(motorR2, 0);
//    analogWrite(motorL1, 0);
//    analogWrite(motorL2, 10);
//    delay(200);
//    stop(false);
//}
//
//void turnLeft(){                                                                              
//    analogWrite(motorR1, 0);
//    analogWrite(motorR2, 50);
//    analogWrite(motorL1, 50);
//    analogWrite(motorL2, 0);
//    delay(200);
//    stop(false);
//}

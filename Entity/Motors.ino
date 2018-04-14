double slowGo(double time){
  return 65*(1-exp(-0.01*time/5));
}

void stop(bool isSpin){    
  if(isSpin){
    double startTime = millis();
    double endTime = 0;
    while (endTime - startTime < 60){
//      digitalWrite(M1Back, HIGH);
//      digitalWrite(M1Front, LOW);
//      analogWrite(M1PWM, 235);
//      digitalWrite(M2Back, HIGH);
//      digitalWrite(M2Front, LOW);
//      analogWrite(M2PWM, 235);
//      digitalWrite(M3Back, HIGH);
//      digitalWrite(M3Front, LOW);
//      analogWrite(M3PWM, 235);
//      digitalWrite(M4Back, HIGH);
//      digitalWrite(M4Front, LOW);
//      analogWrite(M4PWM, 235);    
    analogWrite(motorR1, 235);
    analogWrite(motorR2, 0);
    analogWrite(motorL1, 235);
    analogWrite(motorL2, 0);  
      endTime = millis();
    } 
  }                                                                                  
//      digitalWrite(M1Back, LOW);
//      digitalWrite(M1Front, LOW);
//      digitalWrite(M2Back, LOW);
//      digitalWrite(M2Front, LOW);
//      digitalWrite(M3Back, LOW);
//      digitalWrite(M3Front, LOW);
//      digitalWrite(M4Back, LOW);
//      digitalWrite(M4Front, LOW);
    analogWrite(motorR1, 0);
    analogWrite(motorR2, 0);
    analogWrite(motorL1, 0);
    analogWrite(motorL2, 0); 
}

void go() {
      digitalWrite(M1Back, LOW);
      digitalWrite(M1Front, HIGH);
      analogWrite(M1PWM, 130);
      digitalWrite(M2Back, LOW);
      digitalWrite(M2Front, HIGH);
      analogWrite(M2PWM, 130);
      digitalWrite(M3Back, LOW);
      digitalWrite(M3Front, HIGH);
      analogWrite(M3PWM, 130);
      digitalWrite(M4Back, LOW);
      digitalWrite(M4Front, HIGH);
      analogWrite(M4PWM, 130); 
//    analogWrite(motorR1, 0);
//    analogWrite(motorR2, 65);
//    analogWrite(motorL1, 0);
//    analogWrite(motorL2, 65); 
}

//void back() {
//  analogWrite(motorR1, 45);
//  analogWrite(motorR2, 0);
//  analogWrite(motorL1, 45);
//  analogWrite(motorL2, 0);
//}
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

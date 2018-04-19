void ledsPID(){
  if(abs(Setpoint) != 180){
    (leftOutput > 5)? digitalWrite(ledGreen, HIGH): digitalWrite(ledGreen, LOW); 
    (rightOutput > 5)? digitalWrite(ledRed, HIGH): digitalWrite(ledRed, LOW);
  }
}

void blinkingLEDS(){
  for(int i=0; i<7; i++){
    digitalWrite(ledGreen, HIGH); digitalWrite(ledRed, HIGH); 
    delay(60);
    digitalWrite(ledGreen, LOW); digitalWrite(ledRed, LOW);
    delay(60);
  }
}

void turnOnRedLED(){
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledRed, HIGH);  
  digitalWrite(ledBlue, LOW); 
  delay(3000);
  turnOffLeds();
}
void turnOnGreenLED(){
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledRed, LOW);  
  digitalWrite(ledBlue, LOW); 
  delay(3000);
  turnOffLeds();
}
void turnOnBlueLED(){
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledRed, LOW);  
  digitalWrite(ledBlue, HIGH); 
  delay(3000);
  turnOffLeds();
}
void turnOnLeds(){
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledRed, HIGH);  
  digitalWrite(ledBlue, LOW);
}
void turnOffLeds(){
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledRed, LOW);
  digitalWrite(ledBlue, LOW);
}


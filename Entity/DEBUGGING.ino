void ledsPID(){
//  Serial.println(Setpoint);
//  Serial.println(Input);
//  Serial.print(leftOutput);
//  Serial.print("\t\t\t");
//  Serial.println(rightOutput);
  (leftOutput > 5)? digitalWrite(ledGreen, HIGH): digitalWrite(ledGreen, LOW); 
  (rightOutput > 5)? digitalWrite(ledRed, HIGH): digitalWrite(ledRed, LOW);
}

void blinkingLEDS(){
  for(int i=0; i<7; i++){
    digitalWrite(ledGreen, HIGH); digitalWrite(ledRed, HIGH); 
    delay(60);
    digitalWrite(ledGreen, LOW); digitalWrite(ledRed, LOW);
    delay(60);
  }
}

void turnOnLeds(){
  digitalWrite(ledGreen, HIGH);
}

void turnOff(){
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledRed, LOW);
}


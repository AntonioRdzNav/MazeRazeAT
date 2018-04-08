void ledsPID(){
//  Serial.println(Setpoint);
//  Serial.println(Input);
//  Serial.print(leftOutput);
//  Serial.print("\t\t\t");
//  Serial.println(rightOutput);
  (leftOutput > 5)? digitalWrite(ledLeft, HIGH): digitalWrite(ledLeft, LOW); 
  (rightOutput > 5)? digitalWrite(ledRight, HIGH): digitalWrite(ledRight, LOW);
}

void blinkingLEDS(){
  for(int i=0; i<7; i++){
    digitalWrite(ledLeft, HIGH); digitalWrite(ledRight, HIGH); 
    delay(60);
    digitalWrite(ledLeft, LOW); digitalWrite(ledRight, LOW);
    delay(60);
  }
}


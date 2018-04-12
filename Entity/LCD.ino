void writeIntLCD(int word, int x, int y){
  actualMillis=millis();
  if(actualMillis-lastMillis > 50){
    lcd.clear();   
  }
  lcd.clear();
  lcd.setCursor(x,y);
  lcd.print(word);
  lastMillis=actualMillis;
}

void writeStringLCD(String word, int x, int y){
  actualMillis=millis();
  if(actualMillis-lastMillis > 50){
    lcd.clear();   
  }
  lcd.clear();
  lcd.setCursor(x,y);
  lcd.print(word);
  lastMillis=actualMillis;
}

void readColor(double &R, double &G, double &B){
  digitalWrite(S2,LOW);   // Setting RED (R) filtered photodiodes to be read
  digitalWrite(S3,LOW);
  R = pulseIn(sensorOut, LOW);   // Reading the output frequency
  
  digitalWrite(S2,HIGH);  // GREEN (G)
  digitalWrite(S3,HIGH);
  G = pulseIn(sensorOut, LOW);
 
  digitalWrite(S2,LOW);  // BLUE (B)
  digitalWrite(S3,HIGH);
  B = pulseIn(sensorOut, LOW);
}

void hardCodedCalibration(){
    color_position_arr[0].red = 42.77;
    color_position_arr[0].green = 141.82;
    color_position_arr[0].blue = 108.00;

    color_position_arr[1].red = 134.00;
    color_position_arr[1].green = 93.43;
    color_position_arr[1].blue = 47.57;

    color_position_arr[2].red = 137.3;
    color_position_arr[2].green = 89.67;
    color_position_arr[2].blue = 105.13;

    color_position_arr[3].red = 210.51;
    color_position_arr[3].green = 230.42;
    color_position_arr[3].blue = 178.43;
    
    color_position_arr[4].red = 25.44;
    color_position_arr[4].green = 27;
    color_position_arr[4].blue = 21.83; 
}

// 0 = AT (red, blue, green, black)
// 1 = LARC_fistChallenge (blue)
// 2 = LARC_secondChallenge (red, blue, green)
void calibrarColores(int challenge){
  StackArray <int> colorOptions;
  if(challenge == 0){
     colorOptions.push(4); colorOptions.push(3); colorOptions.push(2); colorOptions.push(1); colorOptions.push(0);
  }
  else if(challenge == 1){
    LARC=true; //Will let the robot go back on spinPID()
    colorOptions.push(1);
  }
  else if(challenge == 2){
    LARC=true; //Will let the robot go back on spinPID()
    colorOptions.push(2); colorOptions.push(1); colorOptions.push(0);
  }
  while(!colorOptions.isEmpty()){
    int i = colorOptions.peek();
    Serial.println("La calibracion del color " + color_names[i] + " iniciara en 5 segundos.");        
    digitalWrite(ledRed, HIGH);
    delay(15000);
//    delay(2000);
    digitalWrite(ledRed, LOW);
    Serial.println("La calibracion del color " + color_names[i] + " ha comenzado.");
    digitalWrite(ledGreen, HIGH);
    delay(800);
    double avr_red = 0, avr_green = 0, avr_blue = 0;
    double COLOR_R, COLOR_G, COLOR_B;
    color_position_arr[i].nombre = color_names[i];
    for(int cont = 0; cont < 1000; cont++){
      readColor(COLOR_R, COLOR_G, COLOR_B);
      avr_red += COLOR_R; avr_green += COLOR_G; avr_blue += COLOR_B;
    }
    color_position_arr[i].red = avr_red / 1000.0; color_position_arr[i].green = avr_green / 1000.0; color_position_arr[i].blue = avr_blue / 1000.0;
    Serial.println("Color = " + color_position_arr[i].nombre + " red = " + color_position_arr[i].red + " green = " + color_position_arr[i].green + " blue = " + color_position_arr[i].blue);
    Serial.println(color_position_arr[i].nombre + " ha sido calibrado");
    digitalWrite(ledGreen, LOW);
    colorOptions.pop();
  }
}

//int currentColor(double r, double g, double b){
int currentColor(){
  readColor(r, g, b);
  Serial.print(r);
  Serial.print("\t");
  Serial.print(g);
  Serial.print("\t");
  Serial.println(b);
  int posicion = -1;
  for(int i = 0; i < num_col; i++){    
    if(r <= color_position_arr[i].red + range && r >= color_position_arr[i].red - range){
      if(g <= color_position_arr[i].green + range && g >= color_position_arr[i].green - range){
        if(b <= color_position_arr[i].blue + range && b >= color_position_arr[i].blue - range){         
          posicion = i;
          return posicion;
        }
      }
    }
  } 
  return posicion;
}

void updateColors(int posicion){
  if(posicion == 3) colorBlackDetected=true; 
  else if(posicion == 0) colorRedDetected = true;
  else if(posicion == 2) colorGreenDetected = true; 
}

void  showSelectedLed(int color){
  if(color == 0){
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledBlue, LOW);  
  }
  else if(color == 1){
    digitalWrite(ledRed, LOW);
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledBlue, HIGH);  
  }
  else if(color == 2){
    digitalWrite(ledRed, LOW);
    digitalWrite(ledGreen, HIGH);
    digitalWrite(ledBlue, LOW);  
  }
}

void stopColor(int color){
  long int temp = millis();  
  while(millis() < temp + 2000){
    stop(false);
    showSelectedLed(color);
  }   
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledBlue, LOW);  
}

bool colorDecision(){
    return (currentColor() != -1); 
}


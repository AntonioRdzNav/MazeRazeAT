//void readColor(double &R, double &G, double &B){
//  digitalWrite(S2,LOW);   // Setting RED (R) filtered photodiodes to be read
//  digitalWrite(S3,LOW);
//  R = pulseIn(sensorOut, LOW);   // Reading the output frequency
//  
//  digitalWrite(S2,HIGH);  // GREEN (G)
//  digitalWrite(S3,HIGH);
//  G = pulseIn(sensorOut, LOW);
// 
//  digitalWrite(S2,LOW);  // BLUE (B)
//  digitalWrite(S3,HIGH);
//  B = pulseIn(sensorOut, LOW);
//}
//
//void calibrarColores(){
//  for(int i = 0; i < num_col; i++){
//    Serial.println("La calibracion del color " + color_names[i] + " iniciara en 3 segundos.");
//    estadoBoton = digitalRead(BotonColores);
//    while(estadoBoton != HIGH){          
//      digitalWrite(LED_red, HIGH);
//      estadoBoton = digitalRead(BotonColores);
//    }
//    digitalWrite(LED_red, LOW);
//    Serial.println("La calibracion del color " + color_names[i] + " ha comenzado.");
//    digitalWrite(LED_blue, HIGH);
//    //delay(1000);
//    double avr_red = 0, avr_green = 0, avr_blue = 0;
//    double COLOR_R, COLOR_G, COLOR_B;
//    color_position_arr[i].nombre = color_names[i];
//    for(int cont = 0; cont < 1000; cont++){
//      readColor(COLOR_R, COLOR_G, COLOR_B);
//      avr_red += COLOR_R; avr_green += COLOR_G; avr_blue += COLOR_B;
//    }
//    color_position_arr[i].red = avr_red / 1000.0; color_position_arr[i].green = avr_green / 1000.0; color_position_arr[i].blue = avr_blue / 1000.0;
//    Serial.println("Color = " + color_position_arr[i].nombre + " red = " + color_position_arr[i].red + " green = " + color_position_arr[i].green + " blue = " + color_position_arr[i].blue);
//    Serial.println(color_position_arr[i].nombre + " ha sido calibrado");
//    digitalWrite(LED_blue, LOW);
//  }
//}
//
//int currentColor(double r, double g, double b){
//  int posicion = -1;
//  for(int i = 0; i < num_col; i++){
//    if(r <= color_position_arr[i].red + range && r >= color_position_arr[i].red - range){
//      if(g <= color_position_arr[i].green + range && g >= color_position_arr[i].green - range){
//        if(b <= color_position_arr[i].blue + range && b >= color_position_arr[i].blue - range){
//          posicion = i;
//          return posicion;
//        }
//      }
//    }
//  }
//  return posicion;
//}


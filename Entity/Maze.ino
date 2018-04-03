//void maze(UltraKalman &ultraRight, UltraKalman &ultraLeft, UltraKalman &ultraFront){  
//  filtrateDistances(ultraRight, ultraLeft, ultraFront);
//  if(!rightSide){ 
//    if(rightDistance < 30){
//      while(!frontSide){                                                              
//        mainPID(linealMovementPID('F')); 
//        filtrateDistances(ultraRight, ultraLeft, ultraFront);
//      }
//    }
//    else{
//      if(special == true){
//        delay(1400);
//        special = false;
//      }
//      filtrateDistances(ultraRight, ultraLeft, ultraFront);
//    }
//    filtrateDistances(ultraRight, ultraLeft, ultraFront);
//    if(leftSide){ 
//      turnRight();
//      back();
//      delay(1700);
//      miniAvanzaBack();
//      calculaDistancia();
//      readColor(r, g, b);
//      showRGB();
//    }
//    else{
//      vueltaDerecha();
//      miniAvanzaFree();
//      readColor(r, g, b);
//      showRGB(); 
//      calculaDistancia();
//      
//    }
//  }
//  else if (!frente){
//    avanza();
//    readColor(r, g, b);
//    special = true;
//    readColor(r, g, b);
//    showRGB();
//    calculaDistancia();
//  }
//  else if(!izquierda){
//    calculaDistancia();
//    if(DISTANCIA_FRENTE < 30){
//      while(!frente){
//        avanza();
//        readColor(r, g, b);
//        showRGB();
//        calculaDistancia();
//      }
//    }
//    else{
//      //delay(200);
//      readColor(r, g, b);
//      showRGB();
//      calculaDistancia();
//    }
//    showRGB(); 
//    calculaDistancia();
//    if(derecha){
//      vueltaIzquierda();
//      back();
//      delay(1700);
//      miniAvanzaBack();
//      calculaDistancia();
//      readColor(r, g, b);
//      showRGB();
//    }
//    else{
//      vueltaDerecha();
//      miniAvanzaFree();  
//      readColor(r, g, b);
//      showRGB(); 
//      calculaDistancia();
//    }
//  }
//  else{
//    showRGB(); 
//    calculaDistancia();
//    vueltaTotal();
//    stop();
//    readColor(r, g, b);
//    showRGB(); 
//    calculaDistancia();
//  }  
//}

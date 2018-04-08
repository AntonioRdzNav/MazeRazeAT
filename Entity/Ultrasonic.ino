void filterUltrasonic(){
  for(int i=0; i<15; i++){
    filtrateDistances(ultraFront, ultraRight, ultraLeft);
  }
}

void rawUltrasonic(UltraKalman &ultra, NewPing ping){
//  double filter = 0;
//  double temp = 0;
//  for(int i = 0; i < 20; i++){
//    temp = ping.ping_cm();
//    if(i != 0 && i != 19){
//      filter += temp;
//    }
//  }
//  ultra.distance = filter / 18;
  ultra.distance = ping.ping_cm();
}


void calculateRawDistances(UltraKalman &ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft){
  rawUltrasonic(ultraRight, pingRight);
  rawUltrasonic(ultraLeft, pingLeft);
  rawUltrasonic(ultraFront, pingFront);
}

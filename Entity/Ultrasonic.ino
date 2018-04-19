void filterUltrasonic(){
  for(int i=0; i<15; i++){
    filtrateDistances(ultraFront, ultraRight, ultraLeft, ultraBack);
  }
}

void rawUltrasonic(UltraKalman &ultra, NewPing ping){
  double ultraTemp = ping.ping_cm();
  if(ultraTemp<200 && ultraTemp!=0){
    if(ultraTemp > 95)
      ultra.distance = 95;
    else
      ultra.distance = ultraTemp;
  }
}


void calculateRawDistances(UltraKalman &ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft, UltraKalman &ultraBack){
  rawUltrasonic(ultraRight, pingRight);
  rawUltrasonic(ultraLeft, pingLeft);
  rawUltrasonic(ultraFront, pingFront);
  rawUltrasonic(ultraBack, pingBack);
}

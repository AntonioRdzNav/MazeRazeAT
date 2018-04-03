void rawRightUltrasonic(UltraKalman &ultraRight){
  ultraRight.distance = pingRight.ping_cm();
}

void rawLeftUltrasonic(UltraKalman &ultraLeft){
  ultraLeft.distance = pingLeft.ping_cm();
}

void rawFrontUltrasonic(UltraKalman &ultraFront){
  ultraFront.distance = pingFront.ping_cm();
}

void calculateRawDistances(UltraKalman &ultraFront, UltraKalman &ultraRight, UltraKalman &ultraLeft){
  rawRightUltrasonic(ultraRight);
  rawLeftUltrasonic(ultraLeft);
  rawFrontUltrasonic(ultraFront);
}

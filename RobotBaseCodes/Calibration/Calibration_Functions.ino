void Calibration_IR(int pin, int ranges[], int arraySize){
  for (int i = 0; i < TEST_NO; i++){
    Serial.flush();
    Serial.print("Place Robot "); Serial.print(ranges[i]); Serial.println("mm Away from a Wall");
    while(Serial.available() == 1){}
    Serial.read();
    Serial.print("Actual Reading: "); Serial.println(analogRead(pin));
    Serial.println(i);
  }
}


void Calibration_US(int ranges[]){
  float val = 0.0;
  for (int i = 0; i < TEST_NO; i++){
    Serial.flush();
    Serial.print("Place Robot "); Serial.print(ranges[i]); Serial.println("mm Away from a Wall");
    while(Serial.available() == 1){}
    Serial.read();
    val = HC_SR04_range();
    Serial.print("Actual Reading: "); Serial.println(val);
    Serial.println();
  }
  
  Serial.println(val);
  while(Serial.available() == 1) { }
}


void Calibration_GYRO(int ranges[]){
}

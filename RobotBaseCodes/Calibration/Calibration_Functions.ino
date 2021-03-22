void Calibration_IR(int pin, int ranges[], int arraySize){
  int val = analogRead(pin);
  for (int i = 0; i < arraySize; i++){
    Serial.flush();
    Serial.print("Place Robot "); Serial.print(ranges[i]); Serial.println("mm Away from a Wall");
    while(Serial.available() == 1){}
    Serial.read();
    Serial.print("Actual Reading: "); Serial.println(val);
    results[i] = val;
  }
}


void Calibration_US(int ranges[], int arraySize){
  float val = 0.0;
  for (int i = 0; i < arraySize; i++){
    Serial.flush();
    Serial.print("Place Robot "); Serial.print(ranges[i]); Serial.println("mm Away from a Wall");
    while(Serial.available() == 1){}
    Serial.read();
    val = HC_SR04_range();
    Serial.print("Actual Reading: "); Serial.println(val);
    results[i] = val;
  }
}


void Calibration_GYRO(int ranges[], int arraySize){
  float val = 0.0;
  for (int i = 0; i < arraySize; i++){
    Serial.flush();
    Serial.print("Place Robot "); Serial.print(ranges[i]); Serial.println("mm Away from a Wall");
    while(Serial.available() == 1){}
    Serial.read();
    val = HC_SR04_range();
    Serial.print("Actual Reading: "); Serial.println(val);
    results[i] = val;
  }
}

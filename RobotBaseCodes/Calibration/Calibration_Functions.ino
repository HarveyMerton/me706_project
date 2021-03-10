void Calibration_IR_Long(){
  generateRanges(100, 800);
  double reading = 9999;
  for (int i = 0; i < TEST_NO+1; i++){
    Serial.print("Place the Robot "); Serial.print(TestingRanges[i]); Serial.println("mm from a Wall.");
    Serial.read();
    //READ VALUE reading = Serial.analogRead(PIN);
    reading = 5;
    Serial.print("Robot Sensor Reading: "); Serial.println(reading); Serial.println();
    SensorReadings[i] = reading;
  }
  printReadings();
}


void Calibration_IR_Short(){
  generateRanges(40, 300);
  double reading = 9999;
  for (int i = 0; i < TEST_NO+1; i++){
    Serial.print("Place the Robot "); Serial.print(TestingRanges[i]); Serial.println("mm from a Wall.");
    Serial.read();
    //READ VALUE reading = Serial.analogRead(PIN);
    reading = 5;
    Serial.print("Robot Sensor Reading: "); Serial.println(reading); Serial.println();
    SensorReadings[i] = reading;
  }
  printReadings();
}


void Calibration_US(){
  generateRanges(20, 1800);
  double reading = 9999;
  HC_SR04_range();
  for (int i = 0; i < TEST_NO+1; i++){
    Serial.print("Place the Robot "); Serial.print(TestingRanges[i]); Serial.println("mm from a Wall.");
    Serial.read();
    reading = HC_SR04_range();
    Serial.print("Robot Sensor Reading: "); Serial.println(reading); Serial.println();
    SensorReadings[i] = reading;
  }
  printReadings();
}


void Calibration_GYRO(){
}

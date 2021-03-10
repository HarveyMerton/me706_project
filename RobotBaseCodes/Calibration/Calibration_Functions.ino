void Calibration_IR_Long(int pin){
  generateRanges(100, 800);
  double reading = 9999;
  for (int i = 0; i < TEST_NO+1; i++){
    Serial.print("Place the Robot "); Serial.print(TestingRanges[i]); Serial.println("mm from a Wall.");
    Serial.read();
    reading = analogRead(pin);
    Serial.print("Robot Sensor Reading: "); Serial.println(reading); Serial.println();
    SensorReadings[i] = reading;
  }
  printReadings();
}


void Calibration_IR_Short(int pin){
  generateRanges(40, 300);
  double reading = 9999;
  for (int i = 0; i < TEST_NO+1; i++){
    Serial.print("Place the Robot "); Serial.print(TestingRanges[i]); Serial.println("mm from a Wall.");
    Serial.read();
    reading = analogRead(pin);
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

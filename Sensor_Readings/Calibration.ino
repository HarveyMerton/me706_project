float Calibration(int pin) {
  for (int i = 0; i < 30; i++) {
    Smoothing(pin);
  }
  return Smoothing(pin);
}

float Calibration_US(int pin) {
  for (int i = 0; i < 30; i++) {
    Smoothing_US(pin);
  }
  return Smoothing_US(pin);
}

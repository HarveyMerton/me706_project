#include <Sensor.h>
int count = 0;
Sensor IR_LF("IRLONG", A5);
Sensor Ultra("ULTRASONIC", 48, 49);
Sensor Gyro("GYROSCOPE", A3);

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println(Gyro.getReading());
  if (count == 1000) {Gyro.setAngle(0); count = 0;}
  count++;
}
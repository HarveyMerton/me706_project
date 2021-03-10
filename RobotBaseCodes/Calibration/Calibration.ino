/* RANGE CALIBRATION FOR IR AND ULTRASONIC SENSORS
 * Author: Peter Mitchell
 * Date: 10 MAR 2021
*/

/* SENSOR ID LIST
 * Sensor Name |  Type  | ID# | Port#
 * ----------------------------------
 *   2D120X_1  | IR Med |  1  |  ?
 *   2D120X_2  | IR Med |  2  |  ?
 *   2Y0A21_1  | IR Lng |  3  |  ?
 *   2Y0A21_2  | IR Lng |  4  |  ?
 *   ADXRS642  | GYRO   |  5  |  ?
 *   HC-SR04   | RADAR  |  6  |  ?
*/

#include <math.h>
#define TEST_NO 5

double TestingRanges[TEST_NO];
double SensorReadings[TEST_NO];

void Calibration();
void Calibration_IR_Short();
void Calibration_IR_Long();
void Calibration_GYRO();
double HC_SR04_range();
void generateRanges(int minRange, int maxRange);
void printReadings();

const unsigned int MAX_DIST = 23200;
const int TRIG_PIN = 9;
const int ECHO_PIN = 8;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.begin(115200);
  Calibration();
}

void loop() {
}

void Calibration(){
    Serial.print("Enter Sensor ID: ");
    int ID =  int(Serial.read());
    ID = 6;
    if (ID == 1 || ID == 2){Calibration_IR_Short();}
    if (ID == 3 || ID == 4){Calibration_IR_Long();}
    else if (ID == 5){Calibration_GYRO();}
    else if (ID == 6){Calibration_US();}
    else {Serial.println("ERROR: ID Not Valid.");}
}


void generateRanges(int minRange, int maxRange){               
  int diff = maxRange - minRange;
  for (int i = 0; i < TEST_NO+1; i++){
    int val = minRange + i*(diff/TEST_NO);
    TestingRanges[i] = val;
  }
}


void printReadings(){
  Serial.println("\n***RESULTS***\n#\tTest  \t  Reading");
  
  for (int i = 0; i < TEST_NO+1; i++){
    Serial.print(i+1);
    Serial.print("\t");
    Serial.print(TestingRanges[i]);
    Serial.print("\t  ");
    Serial.println(SensorReadings[i]);
  }
}

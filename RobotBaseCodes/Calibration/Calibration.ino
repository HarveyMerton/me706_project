/* RANGE CALIBRATION FOR IR AND ULTRASONIC SENSORS
 * Author: Peter Mitchell
 * Date: 10 MAR 2021
*/

/* SENSOR ID LIST
 *   Name   | Type    | ID# | Port#
 * ---------------------------------------
 *   IR_FL  | IR Long |  1  |  A4
 *   IR_FR  | IR Long |  2  |  A5
 *   IR_LF  | IR Med  |  3  |  A6
 *   IR_LR  | IR Med  |  4  |  A7
 *   GYRO   | GYRO    |  5  |  A3
 *   US     | RADAR   |  6  |  T48/E49
*/

#include <math.h>
#include <stdlib.h>

#define TEST_NO 5
#define IR_FL A4
#define IR_FR A5
#define IR_LF A6
#define IR_LR A7
#define GYRO A3
#define TRIG_PIN 48
#define ECHO_PIN 49
#define MAX_DIST 23200

int Ranges_IR_Long [TEST_NO] = {110,175,240,305,370};
int Ranges_IR_Med [TEST_NO] = {210,385,560,735,910};
int Ranges_Gyro [TEST_NO] = {0,90,170,270,360};
int Ranges_Ultrasonic [TEST_NO] = {95,550,1000,1450,1890};

int ID = -1;

int ranges [TEST_NO];
float results [TEST_NO];

void Calibration();
void Calibration_IR(int pin, int ranges[], int arraySize);
void Calibration_US(int ranges[]);
void Calibration_GYRO(int ranges[], int arraySize);
void printResults(int ranges[], int arraySize);
double HC_SR04_range();


void setup() {
  pinMode(IR_FL, INPUT);
  pinMode(IR_FR, INPUT);
  pinMode(IR_LF, INPUT);
  pinMode(IR_LR, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  digitalWrite(TRIG_PIN, LOW);
  
  Serial.begin(9600);
}

void loop() {
  Calibration();
}

void Calibration(){
  goto Error;
  Error:
  Serial.print("Enter Sensor ID: ");
  while (ID == -1){ID = Serial.read();}
  switch(ID){
    case 49: //1
      Serial.println("1");
      Calibration_IR(IR_LF, Ranges_IR_Med, TEST_NO);
      printResults(Ranges_IR_Med, TEST_NO);
      break;
    case 50: //2
      Serial.println("2");
      Calibration_IR(IR_LF, Ranges_IR_Med, TEST_NO);
      printResults(Ranges_IR_Med, TEST_NO);
      break;
    case 51: //3
      Serial.println("3");
      Calibration_IR(IR_LF, Ranges_IR_Long, TEST_NO);
      printResults(Ranges_IR_Long, TEST_NO);
      break;
    case 52: //4
      Serial.println("4");
      Calibration_IR(IR_LF, Ranges_IR_Long, TEST_NO);
      printResults(Ranges_IR_Long, TEST_NO);
      break;
    case 53: //5
      Serial.println("5");
      Calibration_GYRO(Ranges_Gyro, TEST_NO);
      printResults(Ranges_Gyro, TEST_NO);
      break;
    case 54: //6
      Serial.println("6");
      Calibration_US(Ranges_Ultrasonic, TEST_NO);
      printResults(Ranges_Ultrasonic, TEST_NO);
      break;
    default:
      Serial.println("ERROR: ID NOT VALID!");
      ID = -1;
      goto Error;
      break;
  }
  Serial.println();
}

void printResults(int ranges[], int arraySize){
  Serial.println("Range\tReading");
  for (int i = 0; i < arraySize; i++){
    Serial.print(ranges[i]); Serial.print("mm\t"); Serial.println(results[i]);
  }
  Serial.println();
}

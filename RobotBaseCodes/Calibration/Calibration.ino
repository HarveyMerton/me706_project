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
#define IRMed1 A6
#define IRMed2 A7
#define IRLng1 A4
#define IRLng2 A5

double TestingRanges[TEST_NO];
double SensorReadings[TEST_NO];
int ID = -1;

void Calibration();
void Calibration_IR_Long(int pin);
void Calibration_IR_Short(int pin);
void Calibration_GYRO();
double HC_SR04_range();
void generateRanges(int minRange, int maxRange);
void printReadings();

const unsigned int MAX_DIST = 23200;
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  Serial.begin(9600);
  Calibration();
}

void loop() {
}

void Calibration(){
    goto Error;
    Error:
    Serial.print("Enter Sensor ID: ");
    while (ID == -1){ID = Serial.read();}
    switch(ID){
      case 49: //1
        Serial.println("1");
        Calibration_IR_Short(IRMed1);
        break;
      case 50: //2
        Serial.println("2");
        Calibration_IR_Short(IRMed2);
        break;
      case 51: //3
        Serial.println("3");
        Calibration_IR_Long(IRLng1);
        break;
      case 52: //4
        Serial.println("4");
        Calibration_IR_Long(IRLng2);
        break;
      case 53: //5
        Serial.println("5");
        while(1){Calibration_GYRO();}
        break;
      case 54: //6
        Serial.println("6");
        while(1){Calibration_US();}
        break;
      default:
        Serial.println("ERROR: ID NOT VALID!");
        ID = -1;
        goto Error;
        break;
    }
    
    /*
    if (ID == 1 || ID == 2){Calibration_IR_Short();}
    if (ID == 3 || ID == 4){Calibration_IR_Long();}
    else if (ID == 5){Calibration_GYRO();}
    else if (ID == 6){Calibration_US();}
    else {Serial.println("ERROR: ID Not Valid.");}
    */
}


void generateRanges(int minRange, int maxRange){
  /*               
  int diff = maxRange - minRange;
  for (int i = 0; i < TEST_NO+1; i++){
    int val = minRange + i*(diff/TEST_NO);
    TestingRanges[i] = val;
  }
  */
}


void printReadings(){
  /*
  Serial.println("\n***RESULTS***\n#\tTest  \t  Reading");
  
  for (int i = 0; i < TEST_NO+1; i++){
    Serial.print(i+1);
    Serial.print("\t");
    Serial.print(TestingRanges[i]);
    Serial.print("\t  ");
    Serial.println(SensorReadings[i]);
  }
  */
}

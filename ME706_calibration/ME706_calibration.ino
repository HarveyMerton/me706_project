/* 
 *  Sensor calibration code for ME 706 projects
 *  
 *  Authors: Harvey Merton 
 *  Date: 10/03/21
 */


void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  
  GYRO_calibrate();

}

//Gyroscope calibration
double AVcc = 5.0; //Analogue supply voltage
double V_null = 2.5; //Null voltage

void GYRO_calibrate()
{
  Serial.println(analogRead(A3)); //Print Gyro value
}

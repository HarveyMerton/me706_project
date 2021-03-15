/* 
 *  Sensor calibration code for ME 706 projects
 *  
 *  Authors: Harvey Merton 
 *  Date: 10/03/21
 */

#include <Gyro.h>
#include <Math.h>

//PINs setup
#define PIN_GYRO_DAT 3 //Analogue 3 

//Global vars setup
Gyro gyro = Gyro(PIN_GYRO_DAT);

void setup() {
  cli();
   
  //Set up serial output
  Serial.begin(115200);

  //Set up gyro
  gyro.timerSetup();
  gyro.calibrate(); 
  gyro.countOn();
  sei();
}

int i = 0;
int j = 0;
void loop() {
  //For testing the frequency of interrupts
  if(i == 0){
    //testFreq();
    //Serial.println(findOC1A());
  }

  //For testing the ability of the gyro to measure
  //testMeasure();
  asm("nop");
}

//For testing the frequency of the interrupts
void testFreq(){ 
  gyro.countOn();
  delay(1000);
  gyro.countOff();
  i++;
  Serial.print("Number of interrupts: "); 
  Serial.println(j);
}

//For testing the measurement output of the gyro
void testMeasure(){ 
  Serial.print("Start ang: ");
  Serial.println(gyro.getAng());
  delay(2000); 

  gyro.countOn(); 
  delay(1000); 

  gyro.countOff();
  Serial.print("End ang: "); 
  Serial.println(gyro.getAng());
  
  gyro.reset();
  Serial.print("After reset: "); 
  Serial.println(gyro.getAng());
}

//Function to find the correct OC1A setting for a desired sampling frequency
float findOC1A(){ 
  return ((16000000/(1024.0*GYRO_Fs))-1);
}

//Interrupt for tc1 - get gyro value for integration
ISR(TIMER1_COMPA_vect){ 
  //Serial.println(1.0/GYRO_Fs);
  //Serial.println(gyro.getAng() + gyro.readAngRate()*(1.0/GYRO_Fs));
  gyro.setAng(gyro.getAng() + gyro.readAngRate()*(1.0/GYRO_Fs));  
  Serial.println(gyro.getAng()); 
  j++;
}

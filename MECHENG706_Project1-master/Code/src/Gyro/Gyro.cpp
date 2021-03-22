/* 
 *  Gyro.cpp - Class for encapsulating the ADSRX642 gyro functions 
 *  Note that the ISR at the bottom of this class must be declared in the main calling sketch and gyroNAME must be replaced by the name of the name of the declared gyro variable
 *
 *
 *  Author: Harvey Merton 
 *  Date: 12/03/21
 */

#include "Arduino.h" 
#include "Gyro.h"


//Constructor
Gyro::Gyro(int pin){ 
  this->PIN_DATA = pin;
  pinMode(PIN_DATA, INPUT); 

  //timerSetup(); //Set up t/c1 for gyro integration - DOESN'T WORK HERE
  //calibrate(); //Calibrate null voltage for gyro - DOESN'T WORK HERE
  //this->V_NULL = 2.5; //Hard code V_NULL
  
}

//Function to find OCnA in timerSetup to give the correct sampling period
float Gyro::findOCnA(){ 
  return ((16000000/(1024.0*GYRO_Fs))-1);
}

//Function to set up t/c 1 for use by the gyro
void Gyro::timerSetup(){ 
  //Reset registers 
  TCCR3A = 0; 
  TCCR3B = 0;  

  //SET UP TC1 for gyro integration
  //PRR0 &= ~(1<<PRTIM1); //Turn off power reduction for TC1
  
  //Set reset value 
  OCR3A = findOCnA(); //1560 for 10 Hz, 156 for 100 Hz

  //Set timer/counter1 to CTC mode
  TCCR3B |= 1<<WGM32;

  //Set prescalar to 1024 and start clock
  TCCR3B |= (1<<CS32) | (1<<CS30);
  
}


//Function to find and set the null voltage
void Gyro::calibrate()
{
  //Find the null voltage
  long gyroSum = 0;
  
  for(int i = 0; i<100; i++){ //Average readings over 100 values
    gyroSum += analogRead(PIN_DATA);
    delay(5);
  }
  
  this->V_NULL = (gyroSum/100)*(AV_CC/ADC_MAX); //Calculate the null voltage from average of readings
  
}

//Read the gyro and return the currently read angular rate (in deg per sec)
float Gyro::readAngRate(){ 
  float angRate = (analogRead(PIN_DATA)*(AV_CC/ADC_MAX)-V_NULL)/GYRO_SENS;

  //Threshold - read zero below threshold to counteract noise
  if(abs(angRate) < GYRO_RATE_THRESHOLD){ 
    angRate = 0;
  }
  
  return angRate;
}

//Read the current predicted angle of the gyro since reset
float Gyro::getAng()
{ 
  return ang;
}

//Set the current predicted angle of the gyro
float Gyro::setAng(float newAng)
{ 
  ang = newAng;
}

//Toggle gyro integration on
void Gyro::countOn(){ 
  TCNT3 = 0;
  TIMSK3 |= 1<<OCIE3A;
}

//Toggle gyro integration off
void Gyro::countOff(){ 
  TIMSK3 &= ~(1<<OCIE3A);
}

//Reset gyro value 
void Gyro::reset(){ 
  this->ang = 0;
}

/*
//Interrupt for tc1 - get gyro value for integration
ISR(TIMER1_COMPA_vect){ 
  gyroNAME.setAng(gyroNAME.getAng() + gyroNAME.readAngRate()*GYRO_Ts);   
}
*/

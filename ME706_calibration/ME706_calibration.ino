/* 
 *  Sensor calibration code for ME 706 projects
 *  
 *  Authors: Harvey Merton 
 *  Date: 10/03/21
 */

#include <avr/io.h>
#include <avr/interrupt.h>

//PINs setup
#define PIN_GYRO_DAT 3 //Analogue 3 

//Gyroscope calibration
#define AV_CC 5.0 //Analogue supply voltage (V)
#define ADC_MAX 1023 //Maximum output value of ADC converter

#define GYRO_SENS 0.007 //Gyro sensitivity (V/dps)
#define GYRO_RATE_THRESHOLD = 1.5 //Rotation rate less than this will be ignored - drift
#define GYRO_Ts 0.1 //0.1 sec sampling period

float V_NULL = 2.5; //Null voltage (V) - voltage when the gyro reading is 0
long gyroAngle = 0; //Total angle tracked by gyro


void setup() {
  cli();
  
  //SET UP TC1 for gyro integration
  PRR0 &= ~(1<<PRTIM1); //Turn off power reduction for TC1
  
  //Set timer/counter1 to CTC mode
  TCCR1A = 0; 
  TCCR1B |= 1<<WGM12; 
  
  //Set reset value 
  OCR1A = 780; 

  //Set prescalar and start clock
  TCCR1A |= (1<<CS12) | (1<<CS10);

  //Enable interrupt
  TIMSK1 |= 1<<OCIE1A;

  
  //Set up serial output
  Serial.begin(115200);

  //Set up pins 
  pinMode(PIN_GYRO_DAT,INPUT);
  
  //Calibrate
  //Serial.println("Calibrating Gyro");
  //GYRO_calibrate();

  sei();
}

void loop() {
 asm("nop");
}


void gyro_calibrate()
{
  //Find the null voltage
  long gyroSum = 0;
  
  for(int i = 0; i<100; i++){ //Average readings over 100 values
    gyroSum += analogRead(PIN_GYRO_DAT);
    delay(5);
  }
  
  V_NULL = (gyroSum/100)*(AV_CC/ADC_MAX);
  
}


float gyro_currentAngle() 
{ 
  
}


//Toggle gyro integration on
void gyro_countOn(){ 
  TIMSK1 |= 1<<OCIE1A;
}

//Toggle gyro integration off
void gyro_countOff(){ 
  TIMSK1 &= ~(1<<OCIE1A);
}

//Reset gyro value 
void gyro_reset(){ 
  gyroAngle = 0;
}

//Interrupt for tc1 - get gyro value for integration
ISR(TIMER1_COMPA_vect){ 
  gyroAngle += analogRead(PIN_GYRO_DAT)*GYRO_Ts;   ///////HEREEEEEE AND CURRENT ANGLEEEEEEEEE
  Serial.println(gyroAngle);
}

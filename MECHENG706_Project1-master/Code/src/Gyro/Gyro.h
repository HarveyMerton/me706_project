/* 
 *  Gyro.h - Class for encapsulating the ADSRX642 gyro functions 
 *  
 *  Author: Harvey Merton 
 *  Date: 12/03/21
 */

 #ifndef Gyro_h 
 #define Gyro_h

#include "Arduino.h" 
#include <Math.h>

//Gyroscope calibration
#define AV_CC 5.0 //Analogue supply voltage (V)
#define ADC_MAX 1023 //Maximum output value of ADC converter

#define GYRO_SENS 0.007 //Gyro sensitivity (V/dps)
#define GYRO_RATE_THRESHOLD 1.5 //Rotation rate less than this will be ignored - avoid noise
#define GYRO_Fs 100 //Sampling frequency in Hz


class Gyro 
{ 
  public: 
    Gyro(int pin); 
    void calibrate(); 
    
    float readAngRate(); 
    float getAng();
    float setAng(float newAng);

    void countOn();
    void countOff(); 
    void reset();

    void timerSetup();
  private: 
    float findOCnA();

    int PIN_DATA;
    float V_NULL;
    float ang;
};

 #endif

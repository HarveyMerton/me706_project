#import <Math.h>

//Smoothing Filter Variables
const int numReadings = 10; //Smoothing Array
int readings[numReadings];  //Readings from sensor
int readIndex = 0;          //Current reading index
int total = 0;              //Running total
int average = 0;            //Average

//Ultrasonic Variables
const int TRIG_PIN = 48;    //Trig pin
const int ECHO_PIN = 49;    //Echo pin
const int MAX_DIST = 23200; //Maximum reading before error

//Sensor Pinouts
const int IR_FL = A4;       //Front left infrared
const int IR_FR = A5;       //Front right infrared
const int IR_LF = A6;       //Left front infrared
const int IR_LR = A7;       //Lefr rear infrared

float IR_threshold[5] = {-1,-1,-1,-1,-1}; //IR_FR, IR_FL, IR_LF, IR_LR

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
  //Serial.println(IR_FR_fxn());
  Serial.println(IR_FR_fxn());
  //Serial.print(" "); 
  //Serial.println(analogRead(IR_LF));
  delay(500);
}
/*
void readHardCode(){ 
  Serial.flush();
  while(Serial.available() == 1){}
  Serial.read();

  //FR 
  Serial.print("IR_FR: "); 
  Serial.print(Calibration(IR_FR));
  
  //FL
  Serial.print("IR_FL: "); 
  Serial.print(Calibration(IR_FL));
  
  //LF 
  Serial.print("IR_LF: "); 
  Serial.print(Calibration(IR_LF));
  
  //LR
  Serial.print("IR_LR: "); 
  Serial.print(Calibration(IR_LR));

  //US
  Serial.print("Ultrasonic: ");
  Serial.print(HC_SR04_range());
}
*/


void readFxns(){ 
  //F 
  Serial.println("IR_FR: "); 
  Serial.println(IR_FR_fxn());
  delay(500);
  
  //FL
  //Serial.print("IR_FL: "); 
  //Serial.print(IR_FL_fxn());
  /*
  //LF 
  Serial.print("IR_LF: "); 
  Serial.print(IR_LF_fxn());
  
  //LR
  Serial.print("IR_LR: "); 
  Serial.print(IR_LR_fxn());
  */
}

float IR_FR_fxn(){ 
  float reading = analogRead(IR_FR); 
  //Serial.print("Raw reading: "); 
  //Serial.println(reading);
  //Serial.print("Function reading: ");
  
  return 152246.3973*pow(analogRead(IR_FR),-1.184450627);  //OLD FXN: pow(-0.0000265306*reading,3) + pow(0.026087248*reading,2) - 8.483151298*reading + 1096.927934; 
}

/*
float IR_FL_fxn(){ 
  float reading = analogRead(IR_FL); 
  return =-0.0000265306*reading^3 + 0.026087248*reading^2 - 8.483151298*reading + 1096.927934; 
}


float IR_LF_fxn(){ 
  float reading = analogRead(IR_LF);
  return 24986.9915299999*reading^-1.013857328; 
}

float IR_LR_fxn(){ 
  float reading = analogRead(IR_LR);
  return pow(21937.28391*reading,-0.99154474);
}
*/

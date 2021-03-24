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

float IR_threshold[5] = {-1,-1,-1,-1,-1}; //IR_FR, IR_FL, IR_LF, IR_LR, US

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

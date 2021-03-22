/*
  MechEng 706 Base Code

  This code provides basic movement and sensor reading for the MechEng 706 Mecanum Wheel Robot Project

  Hardware:
    Arduino Mega2560 https://www.arduino.cc/en/Guide/ArduinoMega2560
    MPU-9250 https://www.sparkfun.com/products/13762
    Ultrasonic Sensor - HC-SR04 https://www.sparkfun.com/products/13959
    Infrared Proximity Sensor - Sharp https://www.sparkfun.com/products/242
    Infrared Proximity Sensor Short Range - Sharp https://www.sparkfun.com/products/12728
    Servo - Generic (Sub-Micro Size) https://www.sparkfun.com/products/9065
    Vex Motor Controller 29 https://www.vexrobotics.com/276-2193.html
    Vex Motors https://www.vexrobotics.com/motors.html
    Turnigy nano-tech 2200mah 2S https://hobbyking.com/en_us/turnigy-nano-tech-2200mah-2s-25-50c-lipo-pack.html

  Date: 11/11/2016
  Author: Logan Stuart
  Modified: 15/02/2018
  Author: Logan Stuart
*/
#include "src/ServoTimer2/ServoTimer2.h"  //Need for Servo pulse output
#include "src/Gyro/Gyro.h"

//#define NO_READ_GYRO  //Uncomment of GYRO is not attached.
#define NO_HC-SR04 //Uncomment of HC-SR04 ultrasonic ranging sensor is not attached.
//#define NO_BATTERY_V_OK //Uncomment of BATTERY_V_OK if you do not care about battery damage.

//Define pins
#define PIN_GYRO_DAT 3 //Analogue 3 

#define IR_FL A4
#define IR_FR A5
#define IR_LF A6
#define IR_LR A7

//Define gyro object
Gyro gyro = Gyro(PIN_GYRO_DAT);


//State machine states
enum STATE {
  INITIALISING,
  RUNNING,
  STOPPED
};

//Refer to Shield Pinouts.jpg for pin locations

//Default motor control pins
const byte left_front = 46;
const byte left_rear = 47;
const byte right_rear = 50;
const byte right_front = 51;

//Default ultrasonic ranging sensor pins, these pins are defined my the Shield
const int TRIG_PIN = 48;
const int ECHO_PIN = 49;

// Anything over 400 cm (23200 us pulse) is "out of range". Hit:If you decrease to this the ranging sensor but the timeout is short, you may not need to read up to 4meters.
const unsigned int MAX_DIST = 23200;

ServoTimer2 left_font_motor;  // create servo object to control Vex Motor Controller 29
ServoTimer2 left_rear_motor;  // create servo object to control Vex Motor Controller 29
ServoTimer2 right_rear_motor;  // create servo object to control Vex Motor Controller 29
ServoTimer2 right_font_motor;  // create servo object to control Vex Motor Controller 29
ServoTimer2 turret_motor;


int speed_val = 100;
int speed_change;

//Serial Pointer
HardwareSerial *SerialCom;

int pos = 0;
void setup(void)
{
  cli();
  turret_motor.attach(11);
  pinMode(LED_BUILTIN, OUTPUT);

  // Set up IR sensors
  pinMode(IR_FL, INPUT);
  pinMode(IR_FR, INPUT);
  pinMode(IR_LF, INPUT);
  pinMode(IR_LR, INPUT);

  // The Trigger pin will tell the sensor to range find
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Setup the Serial port and pointer, the pointer allows switching the debug info through the USB port(Serial) or Bluetooth port(Serial1) with ease.
  SerialCom = &Serial;
  SerialCom->begin(115200);
  SerialCom->println("MECHENG706_Base_Code_25/01/2018");
  delay(1000);
  SerialCom->println("Setup....");

  //Setup gyro
  gyro.timerSetup();
  gyro.calibrate(); 
  gyro.countOn();

  delay(1000); //settling time but no really needed
  sei();
}

void loop(void) //main loop
{
  static STATE machine_state = INITIALISING;
  //Finite-state machine Code
  switch (machine_state) {
    case INITIALISING:
      machine_state = initialising();
      break;
    case RUNNING: //Lipo Battery Volage OK
      machine_state =  running();
      break;
    case STOPPED: //Stop of Lipo Battery voltage is too low, to protect Battery
      machine_state =  stopped();
      break;
  };
}


STATE initialising() {
  //initialising
  SerialCom->println("INITIALISING....");
  delay(1000); //One second delay to see the serial string "INITIALISING...."
  SerialCom->println("Enabling Motors...");
  enable_motors();
  SerialCom->println("RUNNING STATE...");
  return RUNNING;
}
int turnCounter = 0;
int speed_val_left = 0;
int speed_val_right = 0;

float l_error = 0;
float f_error = 0;
float error_total = 0;

float turn_error = 0;
float turn_error_total = 0;

float l_target = 430; //Decrease = further out, increase = closer //350 380
float f_target = 330; //Decrease = further out, increase = closer //250 280

// Proportional constants for moving straight and turning 90 degrees respectively
float kp_f = 7;
float ki_f = 0.005;

float kp_t = 7;
float ki_t = 0.005;

STATE running() {

  static unsigned long previous_millis;

  //MY CODE

  l_error = l_target - IR_left_calc(analogRead(IR_LF));
  f_error = f_target - IR_front_calc(analogRead(IR_FL));

  if ((turnCounter == 3) && (l_error < 0) && (f_error < 0)) {stop();}

  error_total = error_total + l_error;

  speed_val_left = 1750 - kp_f*l_error - ki_f*error_total;
  speed_val_right = 1250 - kp_f*l_error - ki_f*error_total;

  if (l_error > 100) {
      speed_val_left = 1500 - kp_f*l_error - ki_f*error_total;
      speed_val_right = 1500 - kp_f*l_error - ki_f*error_total;
  }

  // If the front of the robot is close to the wall
  if (f_error < 0) {
    //gyro.countOn();
    gyro.reset();
    while(gyro.getAng() < 85.00){ //ASDF

      // Calculating errors
      turn_error = 90 - gyro.getAng();
      turn_error_total = turn_error_total + turn_error;

      // Using errors to calculate power to send to servos
      speed_val_left = constrain(1500 + kp_t*turn_error + ki_t*turn_error_total, 1250, 2000);
      speed_val_right = constrain(1500 + kp_t*turn_error + ki_t*turn_error_total, 1250, 2000);

      // Writing power to motors
      left_font_motor.write(speed_val_left);
      left_rear_motor.write(speed_val_left);
      right_rear_motor.write(speed_val_right);
      right_font_motor.write(speed_val_right);
    }

    // only change turnCounter on correct turn, where wall to the left of the robot is less than 200mm away
    if (IR_left_calc(analogRead(IR_LF)) < 200) { turnCounter++;} 
    
  } else { //Move forward and keep straight

    speed_val_left = constrain(speed_val_left, 1250, 2000); // keeps left speed between 1250 and 2000 ms
    speed_val_right = constrain(speed_val_right, 1000, 1750); // keeps right side speed between 1000 and 1750 ms

    // Writing power to motors
    left_font_motor.write(speed_val_left);
    left_rear_motor.write(speed_val_left);
    right_rear_motor.write(speed_val_right);
    right_font_motor.write(speed_val_right);

  }

//  Analog_Range_A4();


    SerialCom->print("Error: ");
    SerialCom->print(l_error);
    SerialCom->print(" | Right: ");
    SerialCom->print(speed_val_right);
    SerialCom->print(" | Left: ");
    SerialCom->print(speed_val_left);

    SerialCom->print(" | ");

//  speed_change_smooth()




  

  //read_serial_command();
  fast_flash_double_LED_builtin();

//  if (millis() - previous_millis > 500) {  //Arduino style 500ms timed execution statement
//    previous_millis = millis();

//    SerialCom->println("RUNNING---------");
//    speed_change_smooth();
//    Analog_Range_A4();

//#ifndef NO_READ_GYRO
//    GYRO_reading();
//#endif
//
//#ifndef NO_HC-SR04
//    HC_SR04_range();
//#endif
//
#ifndef NO_BATTERY_V_OK
    if (!is_battery_voltage_OK()) return STOPPED;
#endif


//    turret_motor.write(pos);
//
//    if (pos == 0)
//    {
//      pos = 45;
//    }
//    else
//    {
//      pos = 0;
//    }
//  }

  return RUNNING;
}

//Stop of Lipo Battery voltage is too low, to protect Battery
STATE stopped() {
  static byte counter_lipo_voltage_ok;
  static unsigned long previous_millis;
  int Lipo_level_cal;
  disable_motors();
  slow_flash_LED_builtin();

  if (millis() - previous_millis > 500) { //print massage every 500ms
    previous_millis = millis();
    SerialCom->println("STOPPED---------");


#ifndef NO_BATTERY_V_OK
    //500ms timed if statement to check lipo and output speed settings
    if (is_battery_voltage_OK()) {
      SerialCom->print("Lipo OK waiting of voltage Counter 10 < ");
      SerialCom->println(counter_lipo_voltage_ok);
      counter_lipo_voltage_ok++;
      if (counter_lipo_voltage_ok > 10) { //Making sure lipo voltage is stable
        counter_lipo_voltage_ok = 0;
        enable_motors();
        SerialCom->println("Lipo OK returning to RUN STATE");
        return RUNNING;
      }
    } else
    {
      counter_lipo_voltage_ok = 0;
    }
#endif
  }
  return STOPPED;
}

void fast_flash_double_LED_builtin()
{
  static byte indexer = 0;
  static unsigned long fast_flash_millis;
  if (millis() > fast_flash_millis) {
    indexer++;
    if (indexer > 4) {
      fast_flash_millis = millis() + 700;
      digitalWrite(LED_BUILTIN, LOW);
      indexer = 0;
    } else {
      fast_flash_millis = millis() + 100;
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }
}

void slow_flash_LED_builtin()
{
  static unsigned long slow_flash_millis;
  if (millis() - slow_flash_millis > 2000) {
    slow_flash_millis = millis();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

void speed_change_smooth()
{
  speed_val += speed_change;
  if (speed_val > 1000)
    speed_val = 1000;
}

#ifndef NO_BATTERY_V_OK
boolean is_battery_voltage_OK()
{
  static byte Low_voltage_counter;
  static unsigned long previous_millis;

  int Lipo_level_cal;
  int raw_lipo;
  //the voltage of a LiPo cell depends on its chemistry and varies from about 3.5V (discharged) = 717(3.5V Min) https://oscarliang.com/lipo-battery-guide/
  //to about 4.20-4.25V (fully charged) = 860(4.2V Max)
  //Lipo Cell voltage should never go below 3V, So 3.5V is a safety factor.
  raw_lipo = analogRead(A0);
  Lipo_level_cal = (raw_lipo - 717);
  Lipo_level_cal = Lipo_level_cal * 100;
  Lipo_level_cal = Lipo_level_cal / 143;

  if (Lipo_level_cal > 0 && Lipo_level_cal < 160) {
    previous_millis = millis();
    SerialCom->print("Lipo level:");
    SerialCom->print(Lipo_level_cal);
    SerialCom->print("%");
    // SerialCom->print(" : Raw Lipo:");
    // SerialCom->println(raw_lipo);
    SerialCom->println("");
    Low_voltage_counter = 0;
    return true;
  } else {
    if (Lipo_level_cal < 0)
      SerialCom->println("Lipo is Disconnected or Power Switch is turned OFF!!!");
    else if (Lipo_level_cal > 160)
      SerialCom->println("!Lipo is Overchanged!!!");
    else {
      SerialCom->println("Lipo voltage too LOW, any lower and the lipo with be damaged");
      SerialCom->print("Please Re-charge Lipo:");
      SerialCom->print(Lipo_level_cal);
      SerialCom->println("%");
    }

    Low_voltage_counter++;
    if (Low_voltage_counter > 5)
      return false;
    else
      return true;
  }

}
#endif

#ifndef NO_HC-SR04
void HC_SR04_range()
{
  unsigned long t1;
  unsigned long t2;
  unsigned long pulse_width;
  float cm;
  float inches;

  // Hold the trigger pin high for at least 10 us
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Wait for pulse on echo pin
  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 0 ) {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000)) {
      SerialCom->println("HC-SR04: NOT found");
      return;
    }
  }

  // Measure how long the echo pin was held high (pulse width)
  // Note: the micros() counter will overflow after ~70 min

  t1 = micros();
  while ( digitalRead(ECHO_PIN) == 1)
  {
    t2 = micros();
    pulse_width = t2 - t1;
    if ( pulse_width > (MAX_DIST + 1000) ) {
      SerialCom->println("HC-SR04: Out of range");
      return;
    }
  }

  t2 = micros();
  pulse_width = t2 - t1;

  // Calculate distance in centimeters and inches. The constants
  // are found in the datasheet, and calculated from the assumed speed
  //of sound in air at sea level (~340 m/s).
  cm = pulse_width / 58.0;
  inches = pulse_width / 148.0;

  // Print out results
  if ( pulse_width > MAX_DIST ) {
    SerialCom->println("HC-SR04: Out of range");
  } else {
    SerialCom->print("HC-SR04:");
    SerialCom->print(cm);
    SerialCom->println("cm");
  }
}
#endif

void Analog_Range_A4()
{
  SerialCom->print("Analog Range A4:");
  SerialCom->println(analogRead(A4));
}

#ifndef NO_READ_GYRO
void GYRO_reading()
{
  SerialCom->print("GYRO A3:");
  SerialCom->println(analogRead(A3));
}
#endif

//Serial command pasing
void read_serial_command()
{
  if (SerialCom->available()) {
    char val = SerialCom->read();
    SerialCom->print("Speed:");
    SerialCom->print(speed_val);
    SerialCom->print(" ms ");

    //Perform an action depending on the command
    switch (val) {
      case 'w'://Move Forward
      case 'W':
        forward ();
        SerialCom->println("Forward");
        break;
      case 's'://Move Backwards
      case 'S':
        reverse ();
        SerialCom->println("Backwards");
        break;
      case 'q'://Turn Left
      case 'Q':
        strafe_left();
        SerialCom->println("Strafe Left");
        break;
      case 'e'://Turn Right
      case 'E':
        strafe_right();
        SerialCom->println("Strafe Right");
        break;
      case 'a'://Turn Right
      case 'A':
        ccw();
        SerialCom->println("ccw");
        break;
      case 'd'://Turn Right
      case 'D':
        cw();
        SerialCom->println("cw");
        break;
      case '-'://Turn Right
      case '_':
        speed_change = -100;
        SerialCom->println("-100");
        break;
      case '=':
      case '+':
        speed_change = 100;
        SerialCom->println("+");
        break;
      default:
        stop();
        SerialCom->println("stop");
        break;
    }

  }

}

//----------------------Motor moments------------------------
//The Vex Motor Controller 29 use Servo Control signals to determine speed and direction, with 0 degrees meaning neutral https://en.wikipedia.org/wiki/Servo_control

void disable_motors()
{
  left_font_motor.detach();  // detach the servo on pin left_front to turn Vex Motor Controller 29 Off
  left_rear_motor.detach();  // detach the servo on pin left_rear to turn Vex Motor Controller 29 Off
  right_rear_motor.detach();  // detach the servo on pin right_rear to turn Vex Motor Controller 29 Off
  right_font_motor.detach();  // detach the servo on pin right_front to turn Vex Motor Controller 29 Off

  pinMode(left_front, INPUT);
  pinMode(left_rear, INPUT);
  pinMode(right_rear, INPUT);
  pinMode(right_front, INPUT);
}

void enable_motors()
{
  left_font_motor.attach(left_front);  // attaches the servo on pin left_front to turn Vex Motor Controller 29 On
  left_rear_motor.attach(left_rear);  // attaches the servo on pin left_rear to turn Vex Motor Controller 29 On
  right_rear_motor.attach(right_rear);  // attaches the servo on pin right_rear to turn Vex Motor Controller 29 On
  right_font_motor.attach(right_front);  // attaches the servo on pin right_front to turn Vex Motor Controller 29 On
}
void stop() //Stop
{
  left_font_motor.write(1500);
  left_rear_motor.write(1500);
  right_rear_motor.write(1500);
  right_font_motor.write(1500);
}

void forward()
{
  left_font_motor.write(1500 + speed_val);
  left_rear_motor.write(1500 + speed_val);
  right_rear_motor.write(1500 - speed_val);
  right_font_motor.write(1500 - speed_val);
}

void reverse ()
{
  left_font_motor.write(1500 - speed_val);
  left_rear_motor.write(1500 - speed_val);
  right_rear_motor.write(1500 + speed_val);
  right_font_motor.write(1500 + speed_val);
}

void ccw ()
{
  left_font_motor.write(1500 - speed_val);
  left_rear_motor.write(1500 - speed_val);
  right_rear_motor.write(1500 - speed_val);
  right_font_motor.write(1500 - speed_val);
}

void cw ()
{
  left_font_motor.write(1500 + speed_val);
  left_rear_motor.write(1500 + speed_val);
  right_rear_motor.write(1500 + speed_val);
  right_font_motor.write(1500 + speed_val);
}

void strafe_left ()
{
  left_font_motor.write(1500 - speed_val);
  left_rear_motor.write(1500 + speed_val);
  right_rear_motor.write(1500 + speed_val);
  right_font_motor.write(1500 - speed_val);
}

void strafe_right ()
{
  left_font_motor.write(1500 + speed_val);
  left_rear_motor.write(1500 - speed_val);
  right_rear_motor.write(1500 - speed_val);
  right_font_motor.write(1500 + speed_val);
}

// Two functions that calculate the distance from IR sensors on the front and left side respectively
float IR_front_calc(float analogReading) {
  float voltage = (analogReading/1024)*5;
  float distance = 299.88 * pow(voltage, -1.173);
  return distance;
}

float IR_left_calc(float analogReading) {
  float voltage = (analogReading/1024)*5;
  float distance = 120.8 * pow(voltage, -1.058);
  return distance;
}

//Interrupt for tc1 - get gyro value for integration
ISR(TIMER3_COMPA_vect){ 
  //Serial.println(1.0/GYRO_Fs);
  //Serial.println(gyro.getAng() + gyro.readAngRate()*(1.0/GYRO_Fs));
  gyro.setAng(gyro.getAng() + gyro.readAngRate()*(1.0/GYRO_Fs));  
  //Serial.println(gyro.getAng()); 
  //j++;
}

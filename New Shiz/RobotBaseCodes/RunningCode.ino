//Running Code States
enum RUNNINGSTATE {SETUP, STRAIGHT, ROTATION, STOP};
RUNNINGSTATE RunningState = SETUP;

//Motor Powers
double speed_val_left = 0;
double speed_val_right = 0;

//Running Code Variables
int turnCount = 0;
int frontTarget = 80;
int leftTarget = 285; //inverse response atm
int rotateTarget = 90;
double l_error;
double f_error;
double IR_error;
double error_total;

//Straight Controller
double S_Kp = 0.005;
double S_Ki = 0.0;
double S_Kd = 0.0;

//Forward Controller
double F_Kp = 7.0;
double F_Ki = 0.005;
double F_Kd = 0.0;

//Rotation Controller
double R_Kp = 7.0;
double R_Ki = 0.005;
double R_Kd = 0.0;

float l_target = 150;
float f_target = 150;

float turn_error = 0;
float turn_error_total = 0;

void TaskMain() {
  Serial.println(RunningState);
  switch (RunningState) {
    case SETUP:
      Setup();
      break;
    case STRAIGHT:
      Straight();
      break;
    case ROTATION:
      Rotation();
      break;
    case STOP:
      stop();
      break;
    default:
      break;
  }  
}

void Setup() {
  //Align robot so its 150mm away from the wall on the left and 150mm away from the wall on the back
  stop();
  RunningState = STRAIGHT;
}

void Straight() {
//  double sideError = IR_LF.getReading() - IR_LR.getReading() + (IR_LF.getReading() - leftTarget);
//  double frontError = IR_FR.getReading() - frontTarget;
//
//  speed_val_left = 1500 + (constrain(F_Kp*frontError, 0, 100) + constrain(S_Kp*sideError, -100, 100));
//  speed_val_right = 1500 - (constrain(F_Kp*frontError, 0, 100) - constrain(S_Kp*sideError, -100, 100));

  l_error = IR_LF.getReading() - leftTarget;
  f_error = IR_FL.getReading() - frontTarget;
  IR_error = IR_LR.getReading() - IR_LF.getReading();
  error_total = error_total + l_error;

//  Serial.println(f_error);



  speed_val_left = 1750 - F_Kp*l_error - F_Ki*error_total + S_Kp*IR_error;
  speed_val_right = 1250 - F_Kp*l_error - F_Ki*error_total + S_Kp*IR_error;

  if (l_error > 100) {
      speed_val_left = 1500 - F_Kp*l_error - F_Ki*error_total + S_Kp*IR_error;
      speed_val_right = 1500 - F_Kp*l_error - F_Ki*error_total + S_Kp*IR_error;
  }

    if (f_error < 0) {
      if (turnCount >= 3) {RunningState = STOP;}
      else {RunningState = ROTATION;}
    } else {
      speed_val_left = constrain(speed_val_left, 1250, 2000); // keeps left speed between 1250 and 2000 ms
      speed_val_right = constrain(speed_val_right, 1000, 1750); // keeps right side speed between 1000 and 1750 ms
  
      // Writing power to motors
       SetMotorSpeed(speed_val_left, speed_val_right);
    }
}

void Rotation() {
    
    GYRO.setAngle(0);
    GYRO.initialise();
    
    while(GYRO.getReading() < 85.00){ //ASDF

      // Calculating errors
      turn_error = 90 - GYRO.getReading();
      turn_error_total = turn_error_total + turn_error;

      Serial.println(GYRO.getReading());

      // Using errors to calculate power to send to servos
      speed_val_left = constrain(1500 + R_Kp*turn_error + R_Ki*turn_error_total, 1250, 2000);
      speed_val_right = constrain(1500 + R_Kp*turn_error + R_Ki*turn_error_total, 1250, 2000);

      // Writing power to motors
      SetMotorSpeed(speed_val_left, speed_val_right);
    }
    turnCount++;
    stop();
    RunningState = STRAIGHT;
}

void SetMotorSpeed(double speed_val_left, double speed_val_right) {
  speed_val_left = constrain(speed_val_left, 750, 2250);
  speed_val_right = constrain(speed_val_right, 750, 2250);

  left_font_motor.writeMicroseconds(speed_val_left);
  left_rear_motor.writeMicroseconds(speed_val_left);
  right_rear_motor.writeMicroseconds(speed_val_right);
  right_font_motor.writeMicroseconds(speed_val_right);
}

void GetSensorReadings() {
  //Use in serial plotter with nothing else printing to serial
  SerialCom->print(IR_FL.getReading());
  SerialCom->print(" ");
  SerialCom->print(IR_FR.getReading());
  SerialCom->print(" ");
  SerialCom->print(IR_LF.getReading());
  SerialCom->print(" ");
  SerialCom->print(IR_LR.getReading());+
  SerialCom->print(" ");
  SerialCom->print(ULTRA.getReading());
  SerialCom->print(" ");
  SerialCom->print(GYRO.getReading());
  SerialCom->println();
}

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

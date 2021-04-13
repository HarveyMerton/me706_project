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
double S_Kp = 2.0;
double S_Ki = 0.0;
double S_Kd = 0.0;

//Forward Controller
double F_Kp = 11.0;
double F_Ki = 0.0;
double F_Kd = 0.0;

//Rotation Controller
double R_Kp = 0.0;
double R_Ki = 0.0;
double R_Kd = 0.0;

void TaskMain() {
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

  l_error = leftTarget - IR_LF.getReading();
  f_error = frontTarget - IR_FL.getReading();
  IR_error = IR_LR.getReading() - IR_LF.getReading();
  error_total = error_total + l_error;



  speed_val_left = 1500 - F_Kp*l_error - F_Kp*error_total + S_Kp*IR_error;
  speed_val_right = 1500 - F_Kp*l_error - F_Kp*error_total + S_Kp*IR_error;



  
  //Make sure the left sensors are equal and stop driving forward when the robot front is 150mm away from the wall
  if (f_error > 0) {
    SetMotorSpeed(speed_val_left, speed_val_right);
  }
  else {
    GYRO.initialise();
    stop();
    if (turnCount >= 3) {RunningState = STOP;}
    else {RunningState = ROTATION;}
  }
}

void Rotation() {
  double rotationError = rotateTarget - GYRO.getReading();
  //Rotate 90 degrees and stop
  if (rotationError > 0) {
    cw();
  }
  else {
    turnCount++;
    stop();
    RunningState = STRAIGHT;
  }
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
  SerialCom->print(IR_LR.getReading());
  SerialCom->print(" ");
  SerialCom->print(ULTRA.getReading());
  SerialCom->print(" ");
  SerialCom->print(GYRO.getReading());
  SerialCom->println();
}

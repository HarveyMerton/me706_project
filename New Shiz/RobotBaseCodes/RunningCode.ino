//Running Code States
enum RUNNINGSTATE {SETUP, STRAIGHT, ROTATION, STOP};
RUNNINGSTATE RunningState = SETUP;

//Motor Powers
//double speed_val_left = 0;
//double speed_val_right = 0;

////Running Code Variables
int turnCount = 0;
//int frontTarget = 80;
//int leftTarget = 285; //inverse response atm
//int rotateTarget = 90;
//double l_error;
//double f_error;
//double IR_error;
//double error_total;
//
////Straight Controller
//double S_Kp = 0.005;
//double S_Ki = 0.0;
//double S_Kd = 0.0;
//
////Forward Controller
//double F_Kp = 7.0;
//double F_Ki = 0.005;
//double F_Kd = 0.0;
//
////Rotation Controller
//double R_Kp = 7.0;
//double R_Ki = 0.005;
//double R_Kd = 0.0;
//
//float l_target = 70;
//float f_target = 80;
//
//float turn_error = 0;
//float turn_error_total = 0;

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

int target = 90; //Decrease = further out, increase = closer //350 380
int forward_target = 150; //Decrease = further out, increase = closer //250 280
float k = 15; //30 15 15 15
float ki = 0.005; //0 0.1 0.01 0.005
int error_total = 0;

void Setup() {
  //Align robot so its 150mm away from the wall on the left and 150mm away from the wall on the back
  double errorLeft = IR_LF.getReading() - IR_LR.getReading();
  double errorToWall = IR_LF.getReading() - target;
    if (abs(errorLeft) > 7.5 || IR_LF.getReading() >= 130 || IR_LR.getReading() >= 130) {
        ccw();
    }
    else if (errorToWall > 0) {
      strafe_left();
    }
    else {
      stop();
      RunningState = STRAIGHT;
    }
}

int error = 0;
int f_error = 0;

void Straight() {
 static unsigned long previous_millis;

  //MY CODE

  error = IR_LF.getReading() - target;
  f_error =  IR_FR.getReading() - forward_target;

  
  //error_total = constrain((error_total + error), -20, upper);
   error_total = (error_total + error);

  //Serial.println(error_total);

  int speed_val_left = 1750 - error * k - ki*error_total;
  int speed_val_right = 1250 - error * k - ki*error_total;

  if (error > 5) {

      int speed_val_left = 1500 - error * k - ki*error_total;
      int speed_val_right = 1500 - error * k - ki*error_total;
  }

  
  if (f_error < 0) {
    GYRO.setAngle(0);
    GYRO.initialise();

    if (turnCount >= 3) {
      RunningState = STOP;
    } else {
      RunningState = ROTATION;
    }
      
            
  } else { //Move forward and keep straight

    if (speed_val_left > 2000) {
      speed_val_left = 2000;
    }
  
    if (speed_val_left < 1250) {
      speed_val_left = 1250;
    }
    
    if (speed_val_right > 1750) {
      speed_val_right = 1750;
    }
  
    if (speed_val_right < 1000) {
      speed_val_right = 1000;
    }

  
    left_font_motor.writeMicroseconds(speed_val_left);
    left_rear_motor.writeMicroseconds(speed_val_left);
  
    right_rear_motor.writeMicroseconds(speed_val_right);
    right_font_motor.writeMicroseconds(speed_val_right);

  }
}

void Rotation() {

      if (GYRO.getReading() < 85.00){ //ASDF
          Serial.println(GYRO.getReading());
          left_font_motor.writeMicroseconds(1500 + 250);
          left_rear_motor.writeMicroseconds(1500 + 250);
          right_rear_motor.writeMicroseconds(1500 + 250);
          right_font_motor.writeMicroseconds(1500 + 250);
          Serial.println(GYRO.getReading());
      } 
      else {
          turnCount++;
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
  SerialCom->print(IR_LR.getReading());+
  SerialCom->print(" ");
  SerialCom->print(ULTRA.getReading());
  SerialCom->print(" ");
  SerialCom->print(GYRO.getReading());
  SerialCom->println();
}

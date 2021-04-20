//Running Code States
enum RUNNINGSTATE {SETUP, STRAIGHT, ROTATION, STOP};
RUNNINGSTATE RunningState = SETUP;

//Motor Powers
double speed_val_left = 0;
double speed_val_right = 0;
double speed_val_rotate = 0;
double speed_val_setup = 150;

//Power Allocation
double max_power_staight = 250;
double min_power_straight = 100;
double max_power_side = 250;
double power_difference = 100;
double max_power_rotate = 250;
double min_power_rotate = 20;

//Running Code Variables
int turnCount = 0;
int frontTarget = 140;
int sideTarget = 90;
int rotateTarget = 90;
double sideError;
double sideErrorTotal = 0; 
double frontError;
double differenceError;
double differenceErrorTotal = 0; 
double rotateError;

double intDistSide = 20; //For ant-integrator windup
double intDistDiff = 20;

double sideErrorExit = 2; //For exiting setup
double differenceErrorExit = 2;

//Controller Gains
double F_Kp = 4;  //Forward Controller
double S_Kp = 7; //Side Controller
double S_Ki = 0.1;
double D_Kp = 4; //Difference Controller //2, 5,4 
double D_Ki = 2; //1, 
double R_Kp = 10;  //Rotation Controller


void TaskMain() {
  Serial.println(RunningState);
  
  //FSM for running all tasks required
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
  //Aligns the side of the robot with the wall
  //speed_val = speed_val_setup;  //Increases motor speed

  //Error calculation
  sideError = IR_LF.getReading() - (sideTarget);
  if(abs(sideError) < intDistSide){sideErrorTotal += sideError;} // Anti-integrator windup
  
  differenceError = IR_LF.getReading() - IR_LR.getReading();
  if(abs(differenceErrorTotal) < intDistDiff){differenceErrorTotal += differenceError;} // Anti-integrator windup
  
  double wallPower = constrain(S_Kp*sideError + S_Ki*sideErrorTotal, -50, 50); //250
  double diffPower = constrain(D_Kp*differenceError + D_Ki*differenceErrorTotal, -250, 250);

  //speed_val = 1500 + power_front - power_side;

  left_font_motor.writeMicroseconds(1500 - wallPower - diffPower);
  left_rear_motor.writeMicroseconds(1500 + wallPower - diffPower);
  right_rear_motor.writeMicroseconds(1500 + wallPower - diffPower);
  right_font_motor.writeMicroseconds(1500 - wallPower - diffPower);

  Serial.println(wallPower); /// CURRENTLY HERE FOR TESTING
  Serial.println(diffPower); 

  //if (differenceError > 20) return ccw();
  //if (sideError > 0) return strafe_left();
  if (abs(differenceError) < differenceErrorExit && abs(sideError) < sideErrorExit) { //&& abs(differenceError) < dfferenceErrorExit) { //UNCOMMENTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT
    RunningState = STRAIGHT;
    sideErrorTotal = 0; 
    differenceErrorTotal = 0; 
  }

  //Logic to get robot properly aligned for driving straight
//  if (IR_LF.getReading() > 130 && IR_LR.getReading() > 130) {strafe_left();}
//  else if (sideError > 5 && abs(differenceError) < 10) {strafe_left();}
//  else if (differenceError > 2) {ccw();}
//  else if (differenceError < -2) {cw();}
//  else {
//    //FSM state exit condition
//    stop();
//    RunningState = STRAIGHT;
//  }
}


void Straight() {
  //Implements three different P controllers in order to drive the robot straight with minimal deviation
  //Error calculation
  frontError = IR_FR.getReading() - frontTarget;
  sideError = IR_LF.getReading() - sideTarget;
  differenceError = IR_LF.getReading() - IR_LR.getReading();

  //Controller power calculation
  double power_front = constrain(F_Kp*frontError, min_power_straight, max_power_staight);
  double power_side = constrain(S_Kp*sideError, -1*max_power_side, max_power_side) + constrain(D_Kp*differenceError, -1*power_difference, power_difference);

  //Motor speeds
  speed_val_left = 1500 + power_front - power_side;
  speed_val_right = 1500 - power_front - power_side;

  Serial.print(power_side); Serial.print(" | "); Serial.println(speed_val_right);
  
  if (frontError > 0) {
    SetMotorSpeed(speed_val_left, speed_val_right);
  }
  else {
    //FSM state exit condition
    stop();
    GYRO.setAngle(0);
    GYRO.initialise();
    if (turnCount >= 3) {RunningState = STOP;}
    else {RunningState = ROTATION;}   
  }
}


void Rotation() {
  //Rotation P controller used to rotate the robot 90 degrees
  rotateError = rotateTarget - GYRO.getReading();
  if (rotateError > 5){
    speed_val_rotate = constrain(1500 + R_Kp*rotateError, 1500 + min_power_rotate, 1500 + max_power_rotate);
    SetMotorSpeed(speed_val_rotate, speed_val_rotate);
  }
  else {
    //FSM state exit condition
    turnCount++;
    RunningState = SETUP;
  }
}


void SetMotorSpeed(double speed_val_left, double speed_val_right) {
  //Function to set robot motor speeds for the left and the right motors
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

float deg=45; // Rotation degree
float s = 0;  //Encoder counts
int sm1 = 0;  //Built-in chanel 1
int sm2 = 0;  //Built-in chanel 2
int r = 0;    //indicator for reading builtin encoder to avoid the reading redundancy
float er;     //Proportional error for PI controller
float eri;    //Integral error for PI controller

int t=0;    //time in ms
int t0=0;   //memory for time in ms

int finish=0;  //finish indicator
int rep=1;     //Repetition indicator

int pos1 = 0;
int pos2 = 0;

float kp = .6*90/deg; //proportional gain of PI
float ki = .02; //integral gain of PI 

//Array to store cut-off values (white if sensor reads below, black if sensor reads above)
//MSB  A0  A1  A2  A3  A4  LSB
int fence[5] = {45, 42, 43, 45, 46};


/*
 * Setup
 */

void setup() {
  
  Serial.begin(250000); //Baud rate of communication 

  Serial.println("Enter the desired rotation in degree.");  
  
  while (Serial.available() == 0) //Obtaining data from user
  { 
    //Wait for user input
  }  
  
  deg = Serial.readString().toFloat(); //Reading the Input string from Serial port.
  if (deg<0)
  {
    analogWrite(3,255);  //change the direction of rotation by applying voltage to pin 3 of arduino
  }
  deg=abs(deg);   

  //Take an initial position reading
  pos1 = GrayToBinary(readSensors());
}


/* 
 *  Helper functions
 */

//Function to convert a Gray code input to a binary/decimal number
int GrayToBinary(int g) {
    int b = 0; //Initialise binary number at 0

    while(g != 0) {  
        b ^= g; //Bit b_x are XOR of bits g_x with all bits left of g_x
        g = (g >> 1); ////Bit shifts g to the right every iteration
    }
    return b;
}

//Read all 5 sensors and binary readings (
int readSensors(){

  int output = B0;

  if (analogRead(A1) > fence[0]){
    output |= (1 << 0);
  }
  if (analogRead(A2) > fence[1]){
    output |= (1 << 1);
  }
  if (analogRead(A3) > fence[2]){
    output |= (1 << 2);
  }
  if (analogRead(A4) > fence[3]){
    output |= (1 << 3);
  }
  if (analogRead(A5) > fence[4]){
    output |= (1 << 4);
  }

  return output;
}


//Bubble sort for sorting an array into ascending order
void sort(float a[], int size) {
    for(int i=0; i<(size-1); i++) {
        for(int o=0; o<(size-(i+1)); o++) {
                if(a[o] > a[o+1]) {
                    float t = a[o];
                    a[o] = a[o+1];
                    a[o+1] = t;
                }
        }
    }
}

//Function for finding the average value of an array
float average (float * array, int len)  // assuming array is int.
{
  float sum = 0;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}

//Function for finding if a value exists in an array
int valueinarray(int val, int arr[])
{
    int i;
    for(i = 0; i < 32; i++)
    {
        if(arr[i] == val)
            return 1;
    }
    return 0;

}


/*
 * Main code
 */
int valuesChanged[32] = {99};
int valuesChanges_index = 0;

int count = 0;
float error_total = 0;
int previous_gray = 0;
float errors[10] = {0,0,0,0,0,0,0,0,0,0};
int isOverflow = 0;
int direction_count = 0;

void loop() {   
t=millis();  //reading time
t0=t;  //saving the current time in memory
  while (t<t0+4000 && rep<=10)   //let the code to run for 4 seconds each with repetitions of 10
  {
      //Index count method - increment or decrement segment counter when a change in value is read
      int sensor_reading_gray = readSensors();

      if (previous_gray != sensor_reading_gray) count++;
      if (previous_gray != sensor_reading_gray && GrayToBinary(previous_gray) < GrayToBinary(sensor_reading_gray)) direction_count++;
      if (previous_gray != sensor_reading_gray && GrayToBinary(previous_gray) > GrayToBinary(sensor_reading_gray)) direction_count--;
      
      if (previous_gray != sensor_reading_gray && !valueinarray(GrayToBinary(sensor_reading_gray), valuesChanged)){
    
      valuesChanged[valuesChanges_index] = GrayToBinary(sensor_reading_gray);
      valuesChanges_index++;
  }
  
  previous_gray = sensor_reading_gray;
    
if (t%10==0) //PI controller that runs every 10ms
{ 
    if (s < deg * 114 * 2 / 360)
    {
      er = deg - s *360/228;
      eri = eri + er;
      analogWrite(6, kp * er + ki * eri);
    }

    if (s >= deg * 228/360)
    {
      analogWrite(6, 0);
      eri = 0;
    }
    delay(1);
}

    sm1 = digitalRead(7); //reading chanel 1 
    sm2 = digitalRead(8);  //reading chanel 2

    if (sm1 != sm2 && r == 0) {  //counting the number changes for both chanels
      s = s + 1;
      r = 1;  // this indicator wont let this condition, (sm1 != sm2), to be counted until the next condition, (sm1 == sm2), happens
    }
    if (sm1 == sm2 && r == 1) {
      s = s + 1;
      r = 0;  // this indicator wont let this condition, (sm1 == sm2), to be counted until the next condition, (sm1 != sm2), happens
    }

t=millis();  //updating time
finish=1;  //changing finish indicator

  }

if (finish==1){   

      //Read the final value
      pos2 = GrayToBinary(readSensors());
      

      //this part of the code is for displaying the result
      delay(500); //half second delay
      rep=rep+1;  // increasing the repetition indicator
      //Serial.print("shaft position from optical absolute sensor from home position: ");
      //Serial.println(abs(pos2-pos1));
      
      Serial.print("shaft displacement from optical absolute sensor: "); //Degrees  //every full Revolution of the shaft is associated with 228 counts of builtin 
                                                                         //encoder so to turn it to degre we can use this formula (s * 360 / 228), "s" is the number of  built-in encoder counts
      
      float Error=0;

      //Calculate the number of degrees turned as predicted by each method
      float index_count = (abs(valuesChanges_index) * 360 / 32)-s*360/228;
      float position_count = (abs(pos2-pos1) * 360 / 32)-s*360/228;

      //Output results
      if (abs(index_count) > abs(position_count)){ //If the position count method is more accurate
          errors[rep-1-1] = abs(position_count);
          Error = position_count; 

          Serial.print("(position count) ");
          Serial.println(abs(pos2-pos1) * 360 / 32);

      } else { //If the index count method is more accurate
          errors[rep-1-1] = abs(index_count);
          Error = index_count;
          Serial.print("(index count) ");
          Serial.println(abs(valuesChanges_index) * 360 / 32);
      }

      Serial.print("Shaft displacement from motor's builtin encoder: ");
      Serial.println(s * 360 / 228);  

      Serial.print("Error :");
      Serial.println(Error);  //displaying error
      Serial.println();
      s = 0;
      count = 0;
      finish=0; 
      valuesChanges_index = 0;

      pos1 = pos2;

//Display the average of the errors
if (rep == 11){
  Serial.print("Average: ");
  sort(errors, 10);

  Serial.println(average(errors, 8)); 

  Serial.print("direction: ");
  Serial.println(direction_count>0?"CW (+)":"CCW (-)");
}
      
}
  analogWrite(6,0);  //turning off the motor
}

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




void setup() {
  
  Serial.begin(250000);                                                 //Baud rate of communication 

  Serial.println("Enter the desired rotation in degree.");  
  
  while (Serial.available() == 0)                                       //Obtaining data from user
  { 
    //Wait for user input
  }  
  
  deg = Serial.readString().toFloat(); //Reading the Input string from Serial port.
  if (deg<0)
  {
    analogWrite(3,255);                                                   //change the direction of rotation by applying voltage to pin 3 of arduino
  }
  deg=abs(deg);   

  pos1 = GrayToBinary(readSensors());
}


float kp = .6*90/deg;                         //proportional gain of PI
float ki = .02;                               //integral gain of PI 


//          MSB  A0  A1  A2  A3  A4  LSB
int fence[5] = {40,39,38,41,41};
//int fence[5] = {45,48,45,45,46};
//
//int GrayToBinary(int num)
//{
//    num ^= num >> 16;
//    num ^= num >>  8;
//    num ^= num >>  4;
//    num ^= num >>  2;
//    num ^= num >>  1;
//    return num;
//}

//Function to convert a Gray code input to a binary/decimal number
//int GrayToBinary(int g){ 
//  int b = 0; //Initialise binary number at 0
//
//  for(; g; g>>1){ //While g!=0, bit shifts g to the right every iteration 
//    b ^= g; //Bit b_x are XOR of bits g_x with all bits left of g_x
//    Serial.println(b);
//  }
//    return b;
//}

int GrayToBinary(int g) {
    int b = 0; //Initialise binary number at 0

    while(g != 0) {  
        b ^= g; //Bit b_x are XOR of bits g_x with all bits left of g_x
        g = (g >> 1); ////Bit shifts g to the right every iteration
    }
    return b;
}
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

int count = 0;

float error_total = 0;

int previous_gray = 0;

float errors[10] = {0,0,0,0,0,0,0,0,0,0};

int isOverflow = 0;

int direction_count = 0;


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

float average (float * array, int len)  // assuming array is int.
{
  float sum = 0;  // sum will be larger than an item, long for safety.
  for (int i = 0 ; i < len ; i++)
    sum += array [i] ;
  return  ((float) sum) / len ;  // average will be fractional, so float may be appropriate.
}

int valuesChanged[32] = {99};

int valuesChanges_index = 0;

int valueinarray(int val, int arr[])
{
    int i;
    for(i = 0; i < 32; i++)
    {
        if(arr[i] == val)
            return 1;
    }
    return 0;

//  for(int i = 0; 
}
void loop() {
  // put your main code here, to run repeatedly:


    
t=millis();                 //reading time
t0=t;                       //sving the current time in memory
  while (t<t0+4000 && rep<=10)                                        //let the code to ran for 4 seconds each with repetitions of 10
  {

      int sensor_reading_gray = readSensors();

 // Serial.println(sensor_reading_gray);

  if (previous_gray != sensor_reading_gray) count++;
  if (previous_gray != sensor_reading_gray && GrayToBinary(previous_gray) < GrayToBinary(sensor_reading_gray)) direction_count++;
  if (previous_gray != sensor_reading_gray && GrayToBinary(previous_gray) > GrayToBinary(sensor_reading_gray)) direction_count--;

  if (previous_gray != sensor_reading_gray && !valueinarray(GrayToBinary(sensor_reading_gray), valuesChanged)){
    
    valuesChanged[valuesChanges_index] = GrayToBinary(sensor_reading_gray);
    valuesChanges_index++;
  }
  
  previous_gray = sensor_reading_gray;
    
if (t%10==0)                                      //PI controller that runs every 10ms
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

    sm1 = digitalRead(7);         //reading chanel 1 
    sm2 = digitalRead(8);         //reading chanel 2

    if (sm1 != sm2 && r == 0) {                                      //counting the number changes for both chanels
      s = s + 1;
      r = 1;                                                         // this indicator wont let this condition, (sm1 != sm2), to be counted until the next condition, (sm1 == sm2), happens
    }
    if (sm1 == sm2 && r == 1) {
      s = s + 1;
      r = 0;                                                         // this indicator wont let this condition, (sm1 == sm2), to be counted until the next condition, (sm1 != sm2), happens
    }

t=millis();           //updating time
finish=1;             //cghanging finish indicator

  }

if (finish==1){   


      pos2 = GrayToBinary(readSensors());
      //Serial.print("pos2: ");
      //Serial.println(pos2);
      

  //this part of the code is for displaying the result
      delay(500);                              //half second delay
      rep=rep+1;                               // increasing the repetition indicator
      //Serial.print("shaft position from optical absolute sensor from home position: ");
      //Serial.println(abs(pos2-pos1));

      
      
      
                                   Serial.print("shaft displacement from optical absolute sensor: "); //Degrees                //every full Revolution of the shaft is associated with 228 counts of builtin 
                                                                          //encoder so to turn it to degre we can use this formula (s * 360 / 228), "s" is the number of  built-in encoder counts
      
      float Error=0;

      float index_count = (abs(valuesChanges_index) * 360 / 32)-s*360/228;
      float position_count = (abs(pos2-pos1) * 360 / 32)-s*360/228;

      if (abs(index_count) > abs(position_count)){
          errors[rep-1-1] = abs(position_count);
          Error = position_count;

Serial.print("(position count) ");
      Serial.println(abs(pos2-pos1) * 360 / 32);

      } else {
          errors[rep-1-1] = abs(index_count);
          Error = index_count;
          Serial.print("(index count) ");
              Serial.println(abs(valuesChanges_index) * 360 / 32);
      }

       
      
      Serial.print("Shaft displacement from motor's builtin encoder: ");
      Serial.println(s * 360 / 228);  


      //errors[rep-1-1] = abs(Error);
      Serial.print("Error :");
      Serial.println(Error);                                              //displaying error
      Serial.println();
      s = 0;
      count = 0;
      finish=0; 
      valuesChanges_index = 0;

      pos1 = pos2;

if (rep == 11){
  Serial.print("Average: ");
  sort(errors, 10);

  
  Serial.println(average(errors, 8)); 
//
//  for (int i=0;i<10;i++){
//
//Serial.println(errors[i]);
//    
//  }

      Serial.print("direction: ");
      Serial.println(direction_count>0?"CW (+)":"CCW (-)");
}
      
}
analogWrite(6,0);                                                         //turning off the motor
}

/*GLOBAL VARIABLES*/
//TIMER VARIABLES//
int b=0;           //Main Loop timer
int c=0;           //Loop Start Time
int t0;            //Program Begin Time

//ENCODER COUNT VARIABLES//
float s = 0;       //Internal Encoder Counts
float s_2;         //Internal Encoder Counts For RPM Of PI Controller
int count = 0;     //External Encoder Counts
int r = 0;         //Internal Encoder Count Repetition Flag

//ENCODER VARIABLES//
int s1 = 0;        //Internal Encoder Channel 1
int s2 = 0;        //Internal Encoder Channel 2
int encA = 0;      //External Encoder Channel A
int encB = 0;      //External Encoder Channel B
int binA = 0;      //External Encoder Binary Channel A
int binB = 0;      //External Encoder Binary Channel B
int binAPrev = 0;  //Previous External Encoder Binary Channel A
int binBPrev = 0;  //Previous External Encoder Binary Channel B
int s2m = 0;       //Internal Encoder Channel 2 Memory
int ThA = 955;     //Threshold for External Encoder Channel A
int ThB = 925;     //Threshold for External Encoder Channel B

//RPM VARIABLES//
float rpmm;        //5 Second RPM Of Internal Encoder

//DIRECTION VARIBLES//
int directionm = 0;   //Internal Encoder Direction Count
int dirm;             //Internal Encoder Direction
int dir = 0;          //External Encoder Direction (0 = CCW, 1 = CW)

//PI CONTROLLER VARIABLES//
float ctrl;        //PI Controller Output
float kp = 0.4;    //Proportional Gain Of PI Controller 
float ki = 0.01;   //Integral Gain Of PI Controller
float eri;         //Integral Of Error Of PI Controller
int repc = 1;      //PI Controller Repetition Flag
int RPM;           //Controller RPM`

//LOOP VARIABLES//
int exitt = 0;       //Loop Exit Condition
int repeat = 0;      //Repeat Flag For Display Update


void setup() {
  Serial.begin(250000);
  Serial.println("Enter the desired RPM.");  
  
  while (Serial.available() == 0) {}    //Wait For User Input 
  RPM = Serial.readString().toFloat();  //Read User Input
  if (RPM < 0) {analogWrite(3, 255);}   //Motor Rotation
  RPM = abs(RPM);
}


void loop() {
  b = millis();    //Read Time
  c = b;           //Store Current Time

  /*MAIN LOOP 15 SECOND RUN*/
  while ((b >= c) && (b <= (c+15500)) && exitt == 0) {
    /*PI CONTROLLER*/
    if (b%13 == 0 && repc == 1) {
      eri = ki * (RPM - rpmm) + eri;
      ctrl = 50 + kp * (RPM - rpmm) + eri;
      //Serial.print(ctrl);
      analogWrite(6, ctrl);
      repc = 0;
    }

    if(b%13==1){repc=1;}         //Trigger PI Controller For Next Iteration

    /*READ ENCODER VALUES*/
    s1=digitalRead(7);           //Channel 1 Of Internal Encoder
    s2=digitalRead(8);           //Channel 2 Of Internal Encoder
    encA = analogRead(A0);       //Channel A Of External Encoder
    encB = analogRead(A1);       //Channel B Of External Encoder

    /*FITERING*/
    //ENCODER A
    if (encA > ThA) {binA = 1;}   //Channel A Threshold
    else {binA = 0;}
    //ENCODER B
    if (encB > ThB) {binB = 1;}   //Channel B Threshold
    else {binB = 0;}

    /*ENCODER COUNTING*/
    //Find edges and count up on edge
    if (binA != binAPrev) {count++;}
    if (binB != binBPrev) {count++;}

    if (s1 != s2 && r == 0) {
      s = s+1;                    //5 Second Counter
      s_2 = s_2+1;                //PI Controller Counter
      r = 1;                      //Repitition Flag
    }

    if (s1 == s2 && r == 1) {
      s = s+1;                    //5 Second Counter
      s_2 = s_2+1;                //PI Controller Counter
      r = 0;                      //Repetition Flag
    }

    /*UPDATE TIME*/
    b = millis();

    /*PROGRAM BEGIN TIME*/
    if (b%100 <= 1 && repeat == 0) {
      t0 = b;
      repeat = 1;
    }

    /*0.1 SECOND TRIGGER*/
    if (b%100 == 0) {
      Serial.print("time in ms: ");
      Serial.print(b-t0);
      Serial.print("  spontaneous speed from encoder (builtin):  (");
      rpmm=(s_2/(2*114))*600;                //0.1 Second Internal RPM Calculation
      Serial.print(rpmm);
      Serial.println(")");
      s_2 = 0;                               //PI Controller Counter Reset

      /*5 SECOND TRIGGER*/
      if ((b-t0)%5000 == 0) {
        Serial.println();
        Serial.print("RPM from builtin encoder: ");
        Serial.println((s/(228))*12);                              //5 Second Internal RPM Calculation
        Serial.print("RPM from optical quadrature encoder: ");
        //Serial.println((0.3692*count));                          //5 Second External RPM Calculation
        Serial.println((0.375*count)-(0.00636*count-1.72)); 
        Serial.print("Error: ");
        Serial.println(((0.375*count)-(0.00636*count-1.72)) - ((s/(228))*12));  //Error
        Serial.print("direction read by motor's sensor: ");
        if (dirm == 0) {Serial.print("CW");}
        else {Serial.print("CCW");}
        Serial.print("  ,   ");
        Serial.print("direction read by sensor:  ");
        if (dir <= 40) {Serial.print("CW");}
        else {Serial.print("CCW");}
        Serial.println();

        // Serial.println(count);

        s = 0;            //Reset Internal Encoder Count
        count = 0;        //Reset External Encoder Count
        directionm = 0;   //Reset Internal Encoder Direction
        dir = 0;          //Reset External Encoder Direction
      }
      delay(1);
    }

    /*DIRECTION*/
    //Built In Encoder
    if((s1 == HIGH) && (s2 == HIGH) && (s2m == LOW)) {directionm = directionm + 1;}
    if((s1 == LOW) && (s2 == LOW) && (s2m == HIGH)) {directionm = directionm + 1;}
    if (directionm > 100) {dirm = 0;}
    if (directionm < 20) {dirm = 1;}

    //External Encoder
    if((binA == HIGH) && (binB == HIGH) && (binBPrev == LOW)) {dir++;}
    if((binA == LOW) && (binB == LOW) && (binBPrev == HIGH)) {dir++;}

    /*ENCODER CHANNEL PREVIOUS READING*/
    s2m = s2;            //Previous Encoder Channel 2
    binAPrev = binA;     //Previous Encoder Channel A
    binBPrev = binB;     //Previous Encoder Channel B

    b = millis();        //Time Update
  }
  analogWrite(6, 0);     //Turning Off The Motor
  exitt = 1;             //Changing The Exit Condition To Prevent The Motor To Run After 15s
}

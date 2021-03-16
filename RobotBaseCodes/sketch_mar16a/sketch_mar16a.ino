int IR_FL = A4;
int IR_FR = A5;
int IR_LF = A6;
int IR_LR = A7;


void setup() {
  // put your setup code here, to run once:
  pinMode(IR_FL, INPUT);
  pinMode(IR_FR, INPUT);
  pinMode(IR_LF, INPUT);
  pinMode(IR_LR, INPUT);

  Serial.begin(9600);  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("IR_FL: ");
  Serial.println(analogRead(IR_FL));

  Serial.print("IR_FR: ");
  Serial.println(analogRead(IR_FR));

  Serial.print("IR_LF: ");
  Serial.println(analogRead(IR_LF));

  Serial.print("IR_LR: ");
  Serial.println(analogRead(IR_LR));

  Serial.println();

  delay(500);
}

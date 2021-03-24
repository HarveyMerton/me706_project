/*GLOBAL VARIABLES*/
int duty = 150;
int timer = 0;
int encA = 0;
int encB = 0;

void setup() {
  Serial.begin(9600);	 //Serial communication on baud rate of 9600
  analogWrite(6, duty);  //Start motor

  //Run motor for 5 seconds to see the behaviour of phototransistor A in response to the encoder disk
  while (timer <= 5000) {
    timer = millis();
    encA = analogRead(A0);
    Serial.println(encA);
  }

  delay(500);	//Visible delay for plotter
  
  //Run motor for 5 seconds to see the behaviour of phototransistor B in response to the encoder disk
  while (timer <= 10500) {
    timer = millis();
    encB = analogRead(A1);
    Serial.println(encB);
  }

  analogWrite(6, 0);	//Stop motor
}


void loop() {/*NULL*/}
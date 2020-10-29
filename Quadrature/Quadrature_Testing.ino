int duty = 150;
int timer = 0;
int encA = 0;
int encB = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  analogWrite(6, duty);

  while (timer <= 5000) {
    timer = millis();
    encA = analogRead(A0);
    Serial.println(encA);
  }

  delay(500);
  
  while (timer <= 10500) {
    timer = millis();
    encB = analogRead(A1);
    Serial.println(encB);
  }

  analogWrite(6, 0);
}


void loop() {
  // put your main code here, to run repeatedly:

}

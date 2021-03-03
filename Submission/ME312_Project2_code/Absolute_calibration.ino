void setup() {
  Serial.begin(250000);//Baud rate of communication 

  int values[5] = {0};

  //When press any key
  while (Serial.available() == 0)   {   }  

  //Read and output the values of all phototransistors to allow calibration
  Serial.println("Read values: ");
      
  values[0] = analogRead(A1);
  values[1] = analogRead(A2);
  values[2] = analogRead(A3);
  values[3] = analogRead(A4);
  values[4] = analogRead(A5);

  for(int i = 0; i < 5; i++) {
    Serial.println(values[i]);
  }
}

void loop() {/*NULL*/}

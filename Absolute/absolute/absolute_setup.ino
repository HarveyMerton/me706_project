void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);//Baud rate of communication 

    int whites[5] = {0};

  while (Serial.available() == 0)   {   }  
   
   Serial.println("White: ");
      
        whites[0] = analogRead(A1);
        whites[1] = analogRead(A2);
        whites[2] = analogRead(A3);
        whites[3] = analogRead(A4);
        whites[4] = analogRead(A5);

        for(int i = 0; i < 5; i++) {
              Serial.println(whites[i]);
  }

   

   

  
  
  
}

void loop() {
  // put your main code here, to run repeatedly:

}

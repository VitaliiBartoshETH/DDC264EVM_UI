#include <EEPROM.h>
int addr = 0;


void setup() {
  // initialize serial:
  Serial.begin(9600);
  pinMode(7,OUTPUT);
  pinMode(13,OUTPUT);
 
}

void loop() {
   if (Serial.available()) {

     int inChar = Serial.read();
     //byte buffer[2];
    // Serial.readBytes(buffer, 2);
    EEPROM.write(addr, inChar);
      
      if (inChar == 49)
    {
      digitalWrite(7, HIGH);
      delay(2000);
      digitalWrite(7, LOW);
      delay(100);
      
      //Serial.print("Incomming message ");
      //Serial.println(inChar);
      
    }
   }
    delay(100);
    
  }


  




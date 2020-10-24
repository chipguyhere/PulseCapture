#include "cgh_PulseCapture.h"

PulseCapture IR_receiver(8, 'I');

void setup() {
  Serial.begin(115200);
  IR_receiver.begin();

  // Provide power to an IR receiver sitting in
  // pins D6,D7,D8 (D6=VCC D7=GND D8=output)
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
    
  Serial.println("IR receive active");
}

void loop() {

  byte receivedBitCount;
  
  unsigned long message =  IR_receiver.read(&receivedBitCount);
  if (receivedBitCount==32) {
    Serial.print("IR received: ");
    Serial.println(message, HEX);
  }
  else if (receivedBitCount==1 && message==1) {
    Serial.println("(key held down)");    
  }

}

/*
PulseCapture Copyright 2019-2020, License: GPLv3
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "chipguy_PulseCapture.h"

chipguy_irReceiver IR_receiver(8);

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

  int receivedBitCount;

  // Read a message and bit count (which will be zero if no message is returned)
  unsigned long message =  IR_receiver.read(receivedBitCount);
  if (receivedBitCount==32) {
    Serial.print("IR received: ");
    Serial.println(message, HEX);
  }
  else if (receivedBitCount==1 && message==1) {
    Serial.println("(key held down)");    
  }

}

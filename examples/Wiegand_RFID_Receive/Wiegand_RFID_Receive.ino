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

// This example demonstrates reading from an RFID reader that connects
// via the Wiegand interface over two digital inputs.
// In this example we're using inputs 10 and 11 as digital inputs.
// Wiegand isn't timing critical and doesn't benefit from timer-enhanced input capture.
// Feel free to use any supported pins.

#include "chipguy_PulseCapture.h"

chipguy_WiegandRx wiegand_receiver(10, 11);

void setup() {
  Serial.begin(115200);
  wiegand_receiver.begin();

  // Provide appropriate signals for beep/LED control inputs of your Wiegand RFID reader
  // by connecting the extra wires to digital pins.
  
  // example: low-cost 125KHz RFID reader from Amazon,
  // "UHPPOTE Security RFID EM-ID Card Reader 125KHz Wiegand 26/34 Output for Access Control".
  // Any other reader's wires and appropriate settings almost certainly will vary
  //pinMode(4, OUTPUT);
  //pinMode(5, OUTPUT);
  //pinMode(6, OUTPUT);
  //digitalWrite(4, LOW);  // example: low selects 34-bit Wiegand on a grey wire, high for 26
  //digitalWrite(5, LOW); // example: low selects green LED color on a blue wire, high for red
  //digitalWrite(6, HIGH); // example: high selects beeper off on a yellow wire, low for beep on
    
  Serial.println("Wiegand RFID receive active");
}

void loop() {

  int receivedBitCount;

  // Read a message and bit count (which will be zero if no message is returned)
  unsigned long message =  wiegand_receiver.read(receivedBitCount);

  if (receivedBitCount) {
    // We'll receive 24 or 32 bits from a 26 or 34 bit Wiegand message.
    // The library automatically removes the two parity bits that are not part of the card ID.
    Serial.print(receivedBitCount);
    Serial.print("-bit RFID message received: ");
    Serial.println(message);
  }

}

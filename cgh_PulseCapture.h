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

// This class uses Timer Input Capture or Pin Change Interrupts to listen for:
//  * Infrared input (current version only receives the most common "NEC" 32 bit scheme)
//  * RFID card reader input (two-wire Wiegand interface) on any two pins (26 or 34 bit)
//  * Servo PWM from an RC radio receiver (pulses between 1-2ms, supported capture 0.5-2.5ms)
//  * Limited (but tiny memory footprint) receive-only SoftSerial implementation

// USAGE:
//  TODO: make this simpler for beginners
//  Example of reading IR on pin 8:
//  PulseCapture IR_receiver = PulseCapture(8, 'I');
//  TODO: add a dummy begin() method per Arduino library recommendation
//  How to check for an incoming message:
//   if (IR_receiver.capturedBitCount) { // 0 if no message
//      Serial.println(IR_receiver.capturedMessage, HEX);
//      IR_receiver.capturedBitCount=0; // clears the message, in a clumsy way, TODO: improve
//   }
//  Wiegand Protocol:
//   use '0' as the protocol for the D0 pin,
//   and then create a second EdgeInput using 'W' as the protocol for the D1 pin.
//   Afterward, only interact with the second one.
//   Wiegand messages are only reported when they are 4, 26, or 34 bits long.
//     4 bit messages (typically keypad presses) are reported unchanged.
//     26 and 34 bits (typically card reads) are stripped of two parity bits
//       and are reported as 24 and 32 bit messages.
//     Check capturedBitCount to see what type of message was received.
//
//  PWM: use 'P' as the protocol.
//   Because the accuracy of received PWM is so timing sensitive, using the pins that
//   support hardware timer input capture pin is HIGHLY recommended.
//   capturedMessage contains the width of the last pulse, in microseconds, as long as it
//   was between 500 and 2500 us.  New incoming pulses update this.

// Multiple simultaneous EdgeInput instances on different pins and protocols are fully supported.
// Tested on Arduino Uno, Nano, and Mega.

// PIN COMPATIBILITY:
//  Arduino Uno and Nano use Timer Input Capture for input on pin 8.
//   Pin Change Interrupt capture is supported on all other pins.
//   Timer Input Capture offers the highest capture resolution and tolerance for interrupt
//   latency from other libraries.
//  Arduino Mega supports Timer Input Capture on pins 48, 49.
//   Mega supports Pin Change Interrupt capture on pins 10,11,12,13,48,49,50,51,52,53,A8-A15
//   Other Mega pins are not supported by this library.




// More generically, this class turns a digital signal into a stream of timestamps of "rise and fall"
// and interprets that stream into messages.

// The module works exclusively over interrupts and activates when the PulseCapture constructor
// is called for the first time.  It fully supports multiple simultaneous instances on different pins.
// When it captures something, it reports it by setting
// capturedBitCount to nonzero, and placing the message in capturedMessage.  




// In addition to taking over the Timer1 clock rate, this module services the following interrupts:
//  * Timer1 Capture
//  * PCINT0-2 (pin change interrupts for all pins)
//  * Timer1 Overflow (used by the protocols to detect timeout conditions)
//  * MEGA: Timer4 and Timer5 Capture

#ifndef __cgh_EdgeInput_h_included
#define __cgh_EdgeInput_h_included


#if defined (ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_UNO)
# define model_UNO_NANO
#elif defined (ARDUINO_AVR_MEGA2560)
# define model_MEGA
#elif defined (ARDUINO_AVR_NANO_EVERY)
# define model_EVERY
#else
# error "this sketch hasn't been made compatible with this hardware, use Uno, Nano, or Mega"
#endif


// Represents one protocol listener that turns a stream of timestamped edges into captured message.
class PulseCapture {
public:

  PulseCapture(byte pin, byte _protocol);
  int begin(void);

  // Reads the last message if any, optionally returning the count of bits in the message.
  uint32_t read();
  uint32_t read(uint8_t *bitcount);
  
  uint8_t capturedBitCount=0;
  uint32_t capturedMessage=0;
  uint8_t inword=0;
  uint8_t protocol=0;      // 9=serial cardreader I=IR W=Wiegand1

  void _handle_irq(void *eev);
  PulseCapture *next_pulsecapture_instance=NULL;

  uint8_t *portaddr=0;
  uint8_t mask=0;
  uint8_t lastRead=0;
  uint32_t lastTimestamp=0;
  uint8_t bitsreceived=0;
  uint8_t incard=0;
  uint8_t charbuf=0;
  uint32_t capbuf=0;
  uint16_t begin_status;

};

#endif

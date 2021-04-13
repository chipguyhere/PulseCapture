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
//  * RFID card reader input (two-wire Wiegand interface) on any two supported pins (26 or 34 bit)
//  * Servo PWM from an RC radio receiver (pulses between 1-2ms, supported capture 0.5-2.5ms)
//  * Receive-only SoftSerial implementation

// USAGE:
//  Example of reading IR on pin 8:
//  chipguy_irReceiver ir(8);
//  ir.begin();
//  unsigned long msg = ir.read();
//  Returns the message, or 0 if none, or 1 if it's a "repeat" code.

//  Wiegand Protocol:
//  chipguy_wiegandRx wiegand(8,9);
//  wiegand.begin();
//  unsigned long msg = wiegand.read();
//   Wiegand messages are only reported when they are 4, 26, or 34 bits long.
//     4 bit messages (typically keypad presses) are reported unchanged.
//     26 and 34 bits (typically card reads) are stripped of two parity bits
//       and are reported as 24 and 32 bit messages.
//     Use overloaded read() to get bit count and see what type of message was received.
//
//  Servo PWM: 
//  chipguy_servoPwmRx ch1(8);
//  ch1.begin();
//  int position = ch1.read();
//   Position is returned between 500 and 2500 microseconds, or 0 if no update
//   since the last pulse.  Pulses are not buffered and are overwritten if not read.
//   Because the accuracy of received PWM is so timing sensitive, using the pins that
//   support hardware timer input capture pin is HIGHLY recommended.

//  Serial:
//  chipguy_softSerialRx ser(8, 9600);
//  ser.begin();
//  int c = ser.read();  // returns next character or -1 if none, without blocking.



// Multiple simultaneous EdgeInput instances on different pins and protocols are fully supported.
// Tested on Arduino Uno, Nano, Mega, Nano Every, Leonardo
// Timer Input Capture offers the highest capture resolution and tolerance for interrupt
// latency from other libraries.


// PIN COMPATIBILITY:
//  Arduino Uno and Nano use Timer Input Capture for input on pin 8.
//   Pin Change Interrupt capture is supported on all other pins.
//  Arduino Mega supports Timer Input Capture on pins 48, 49.
//   Mega supports Pin Change Interrupt capture on pins 10,11,12,13,48,49,50,51,52,53,A8-A15
//   Other Mega pins are not supported by this library, they simply won't work.
//  Arduino Nano Every supports Timer Input Capture on all pins.
//   Timer Input Capture is supported on up to almost* any 3 pins
//     (*almost: maximum of two in a single port group.  Port groups are AB, CD, and EF)
//     Pin Change Interrupt also supported on all pins and will be automatically
//     selected as needed.
//  Arduino Leonardo and Micro supports Timer Input Capture on pins 4 and 13.
//   Pin Change Interrupt capture supported on 8,9,10,11,MISO(14),SCK(15),MOSI(16),SS(17)
//   Other pins are not supported and simply won't work.
//   MISO,SCK,MOSI are reached on the ICSP port.  SS may be inaccessible.


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
#elif defined (ARDUINO_AVR_LEONARDO_ETH) || defined(ARDUINO_AVR_LEONARDO) || defined(ARDUINO_AVR_MICRO)
# define model_32U4
#else
# error "this sketch hasn't been made compatible with this hardware, use Uno, Nano, Mega, Micro, or Leonardo"
#endif


// Base class for all PulseCapture classes
// Represents one protocol listener that turns a stream of timestamped edges into captured message.
class PulseCapture {
public:

  PulseCapture();
  PulseCapture(byte pin, byte _protocol);
  
  // return values of begin():
  //  3 : active using hardware timer capture 
  //  2 : active using pin change interrupts
  //  1 : active using classic Arduino attachInterrupt/millis/micros
  // -1 : pin not supported, nothing active
  int begin(void);
  void init(byte _pin, byte _protocol);

  // Reads the last message if any, optionally returning the count of bits in the message.
  uint32_t read();
  uint32_t read(int &bitcount);
  
  uint8_t capturedBitCount=0;
  uint32_t capturedMessage=0;

  void* _handle_irq(void *eev); 
  virtual void _handle_edge(char edgeKind, uint32_t timediff32, uint16_t timediff);    
  uint8_t pin=0;
  uint8_t inword=0;
  uint8_t protocol=0;      // 9=serial cardreader I=IR W=Wiegand1
  char portid=0;
  uint8_t mask=0;
  uint8_t lastRead=0;
  uint32_t lastTimestamp=0;
  uint8_t bitsreceived=0;
  uint32_t capbuf=0;
  uint8_t clockrate=0;  
  PulseCapture *next_pulsecapture_instance=NULL;

};


class chipguy_irReceiver : public PulseCapture {
public:
	chipguy_irReceiver(byte _pin);

	void _handle_edge(char edgeKind, uint32_t timediff32, uint16_t timediff);	
private:

	uint32_t lastMessageTimestamp=0;
	uint32_t sumdiffs=0;

};


class chipguy_softSerialRx : public PulseCapture {
public:
	chipguy_softSerialRx(byte _pin, uint32_t _baud);
		
	// When head==tail the buffer is empty
	byte receive_head=0; // index of the next byte that will be written
	byte receive_tail=0;
	byte receiveBuffer[32];
	
	void _handle_edge(char edgeKind, uint32_t timediff32, uint16_t timediff);	
	int read(void);

private:	
	uint16_t baudmulfac;
  uint8_t charbuf=0;

};


class chipguy_servoPwmRx : public PulseCapture {
public:
	chipguy_servoPwmRx(byte pin);
	void _handle_edge(char edgeKind, uint32_t timediff32, uint16_t timediff);	
	
};

// helper class for WiegandRx
class chipguy_wiegand_helper : public PulseCapture {
public:
	chipguy_wiegand_helper();
	void _handle_edge(char edgeKind, uint32_t timediff32, uint16_t timediff);	
	
	
	PulseCapture* d1instance;
};


class chipguy_WiegandRx : public PulseCapture {
public:
	chipguy_WiegandRx(byte pinD0, byte pinD1);
	void _handle_edge(char edgeKind, uint32_t timediff32, uint16_t timediff);	
  int begin(void);
	
private:
  chipguy_wiegand_helper helper;
};



#endif

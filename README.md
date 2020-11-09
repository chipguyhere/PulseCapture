# PulseCapture

PulseCapture is an interrupt-driven library for Arduino Uno/Nano/Mega that captures several types of pulsed digital signals:

* Infrared (the most common NEC protocol)
* Wiegand (two-wire protocol used in RFID readers)
* Servo PWM with 0.5μs resolution
* Soft Serial Rx

On Uno and Nano, you can use any digital or analog input.  The Pin Change Interrupt is used for capture, which is hardware-supported on all pins.

On Arduino Mega, these pins have hardware interrupt support and are the only ones supported by the library: 10, 11, 12, 13, 48, 49, 50, 51, 52, 53, and A8 thru A15.

On Arduino Nano Every, all pins are supported.

On 32u4 including Leonardo and Micro, only these pins are supported: 4,8,9,10,11,13,MISO,SCK,MOSI (ICSP pins).

This library uses an enhanced Timer Input Capture hardware feature when your input is connected to the specific pin(s) that support it.  This is highly recommended for infrared and servo PWM input, especially on projects that have excessive interrupt latency elsewhere (such as those driving WS281xx LED strips).  Timer input capture provides an enhanced resolution of 0.5μs and eliminates servo jitter.

* On Uno and Nano, pin 8
* On Mega, pins 48 and 49.
* On Nano Every, all pins, up to 3 simultaneous.
* On 32u4, pins 4 and 13.

## How to install:

* Install Arduino IDE (for PC/Mac/Linux) as downloaded from arduino.cc if not already on your computer.
* See that you have an Arduino sketchbook folder already created by default when you first started the Arduino IDE (typically a folder named Arduino in your "Documents" folder.  This folder should contain another automatically-created folder named "libraries" that you can create if it doesn't already exist)
* Download ZIP file from GitHub
* Create a folder at (your Arduino sketchbook folder)/libraries/PulseCapture and unzip file here
* (Re-)start Arduino IDE

## Basic usage:
### General flow

* ```#include "cgh-PulseCapture.h"```
* Create an instance of the class.
* Call ```begin()```
* No further action is needed for the class to receive incoming event messages.
* Poll for incoming events using ```read()```, which always returns immediately, with either a complete message, or a return value of 0 and a bit count of 0 if there is no complete message received.

#### Notes
* An event is the arrival of a complete message (such as a complete IR command, or a complete RFID card swipe, or a complete PWM timing measurement).
* You can detect a received event without removing it from the buffer by checking ```capturedBitCount``` for a non-zero value.
* Each instance will only buffer one complete "message" event.  If a second event finishes arriving before the first event is read, the first event is overwritten and discarded.  This design is intentional.

### Infrared receiver:

```
  chipguy_irReceiver(8);
```
In your ```setup()```:
```
  IR_receiver.begin();
```
In your ```loop()```, you can poll for received IR messages with the ```read()``` function.  A non-zero return value indicates a message received.

```  
  int receivedBitCount;
  unsigned long message =  IR_receiver.read(receivedBitCount);
  if (receivedBitCount==32) {
    Serial.print("IR received: ");
    Serial.println(message, HEX);
  }
  else if (receivedBitCount==1 && message==1) {
    Serial.println("(key held down)");    
  }
```

The "key held down" is a special message in the IR protocol, returned by the class as a single bit message of 1.
The class will not allow the "key held down" message to overwrite any other unread message in the receive buffer.

### Wiegand (RFID) receiver:

Wiegand is a popular protocol for RFID readers.  Most readers that use this protocol will return messages of
26 or 34 bits (which represent 24- and 32-bit ID numbers with parity bits).  This library removes the
parity bits and only returns the 24- or 32-bit ID.  As implemented, it ignores messages of any
other length (other than 4-bit keypress messages).

Wiegand is a one-way protocol, with messages simply arriving to indicate successful RFID card reads.
Since messages are rarely larger than 32 bits, generally only the card serial number is sent (varies by reader).
Most RFID readers send a message once when a card is presented.  Some RFID readers, but not all,
will repeat the message a few times per second while the card remains present in the RFID's reading field.

Some RFID readers have numeric keypads.  These report keypresses in the form of 4-bit messages.
The messages will be the numbers 0 thru F (hex).

The PulseCapture library provides a derived Wiegand class to simplify the creation of PulseCapture on two
simultaneous pins.  Create it as follows:

```
  chipguy_WiegandRx wiegand(4, 5);   // Read Wiegand protocol on pins 4 (Data0) and 5 (Data1)
```
In your ```loop()```, receivedBitCount will indicate 4, 24, or 32 when a message is received, or 0 if none.

```  
  int receivedBitCount;
  unsigned long message =  wiegand.read(receivedBitCount);
  if (receivedBitCount) {
    Serial.print(receivedBitCount);
    Serial.print("-bit message received: ");
    Serial.println(message);
  }
```

## System resources impacted

This library requires complete control of some of your timers in order to work.  Because of that, using PWM as follows will conflict with this library:

* Pins 9 and 10 (Uno and Nano only)
* Pins 11 and 12 (Mega only)
* Pins 6,7,8 while capturing on pin 49 (Mega only)
* Pins 44,45,46 while capturing on pin 48 (Mega only)
* Pins 3 and 6 on Arduino Nano Every
* Pins 5, 9, and 10 on 32u4 including Leonardo and Micro

The following timers get taken:
* Uno/Nano: Timer1
* Mega: Timer4, Timer5
* 32u4/Leonardo/Micro: Timer1, Timer3
* Every: TCB0, TCB1, TCB2

This library also takes one of your Compare Match interrupts on your main system timer (Timer0 or TCA0), though I can't think of what existing libraries this might impact (the timer remains usable for PWM etc.)


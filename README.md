# PulseCapture

PulseCapture is an interrupt-driven library for AVR Arduino boards (Uno/Nano/Mega/Micro/Leonardo)
that uses *hardware timer capture* to capture several types of pulsed digital signals:

* Infrared remote-control signals
* Wiegand (two-wire protocol used in RFID readers)
* Servo PWM with sub-microsecond resolution
* Hardware-assisted Soft Serial Rx, and support for simultaneous active instances (Rx on multiple pins at the same time)

*Hardware timer capture* is a feature of the AVR chips that offloads the time stamping of incoming pulses
to the on-chip *timer* units.  Without this step, other background activities performed while your sketch runs (like
increasing the ```millis()``` count, or checking for incoming serial data)
will add short but frequent unpredictable delays between
receiving a signal and processing it, several times per second, which interferes with reliability and quality.

These delays (especially when observed on servo readings) are sometimes called "jitter".
Running color LED strips from a sketch is a frequent and common cause of extreme jitter.

## Supported models and pins
On Arduino Uno and Nano, you can use any digital or analog input, especially pin __8__.  The Pin Change Interrupt is used for capture, which is hardware-supported on all pins.

On Arduino Mega, only these pins are supported: 10, 11, 12, 13, __48__, __49__, 50, 51, 52, 53, and A8 thru A15.

On Arduino Nano Every, all pins are supported.

On 32u4 including Leonardo and Micro, only these pins are supported: __4__,8,9,10,11,__13__,MISO,SCK,MOSI (ICSP pins).

This library uses an enhanced Timer Input Capture hardware feature when your input is connected to the specific pin(s) that support it.  This is highly recommended for infrared and servo PWM input, especially on projects that have excessive interrupt latency elsewhere (such as those driving WS281xx LED strips).  Servo resolution on non-enhanced pins is limited to 4µs and subject to potential jitter from other onboard interrupts.

* On Uno and Nano, pin 8
* On Mega, pins 48 and 49.
* On Nano Every, all pins.  Full hardware capture available on any 2 pins simultaneously.  Full hardware capture also available on *almost* any 3 pins (see list of exceptions).
* On 32u4, pins 4 and 13.

## How to install:

* Install Arduino IDE (for PC/Mac/Linux) as downloaded from arduino.cc if not already on your computer.
* See that you have an Arduino sketchbook folder already created by default when you first started the Arduino IDE (typically a folder named Arduino in your "Documents" folder.  This folder should contain another automatically-created folder named "libraries" that you can create if it doesn't already exist)
* Download ZIP file from GitHub
* Create a folder at (your Arduino sketchbook folder)/libraries/PulseCapture and unzip file here
* (Re-)start Arduino IDE

## Basic usage:
### General flow

* ```#include "chipguy-PulseCapture.h"```
* Create an instance of the class.
* Call ```begin()```
* No further action is needed for the class to receive incoming event messages, as all of this processing is handled in the background via interrupts.
* Poll for incoming events using ```read()```, which always returns immediately, with either a complete message, or a return value of 0 and a bit count of 0 if there is not yet a complete message received.

#### Notes
* An event is the arrival of a complete message (such as a complete IR command, or a complete RFID card swipe, or a complete PWM timing measurement).
* You can detect a received event without removing it from the buffer by checking ```capturedBitCount``` for a non-zero value.
* Each instance will only buffer one complete "message" event.  If a second event finishes arriving before the first event is read, the first event is overwritten and discarded.  This design is intentional.
* Return values of ```begin()```: 
  * ```3``` active using hardware timer capture 
  * ```2``` active using pin change interrupts
  * ```1``` active using classic Arduino attachInterrupt/millis/micros
  * ```-1``` pin not supported, nothing active

### Basic usage of Infrared receiver:

```
  chipguy_irReceiver IR_receiver(8);
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

The "key held down" is a special message in the IR protocol, returned by the class as a single bit message of ```1```.  The special "key held down" message will be discarded if the buffer is full, rather than having it overwrite the (likely) unread message it refers to.

### Wiegand (RFID) receiver:

Wiegand is a popular two-wire protocol for RFID readers.  Receiving a message from a card reader is very similar
to receiving one from an infrared receiver -- the message is a number up to 32 bits and simply shows up when the hardware receives it.
[Details](docs/Wiegand.md)


### Servo PWM receiver

The Servo PWM receiver captures servo pulse width as a background task using hardware timer capture.


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

This library also takes one of your Compare Match interrupts on your main system timer (Timer0 or TCA0) (though the timer remains usable for PWM etc.)

### Arduino Every limitation
Arduino Every's "event routing" system has a minor limitation to be aware of.  Only two pins from a single "group" can have hardware timer capture enabled, so if you enable three pins, they must be spread across two or more groups, or else the third pin to call ```begin()``` will revert to non-hardware capture.
* First group (D2, D5, D7, D9, D10)
* Second group (D0, D1, D4, A0, A1, A2, A3, A6, A7)
* Third group (D3, D6, D8, D11, D12, D13, A4, A5)




# PulseCapture

PulseCapture is an interrupt-driven library for Arduino Uno/Nano/Mega that captures several types of timed digital signals:

* Infrared (the most common NEC protocol)
* Wiegand (two-wire protocol used in RFID readers)
* Servo PWM with 0.5μs resolution

On Uno and Nano, pulses can be captured on any digital or analog input (not just pins 2 and 3), as the library uses Pin Change Interrupts to
capture pulses.  On Mega, this library supports capture only on these hardware-supported pins: 10, 11, 12, 13, 48, 49, 50, 51, 52, 53, and A8 thru A15.

This library uses hardware-based Timer Input Capture when your input is connected to the specific pin(s) that support it.
These provide the highest resolution and are the best pins to use with this library.  On Uno and Nano, that's pin 8.  On Mega, that's pins 48 and 49.

Hardware input capture uses a 16-bit timer to timestamp the incoming pulses when they arrive, providing a resolution of 0.5μs that is unaffected by other interrupts running
on the microcontroller.  This is especially important when receiving servo PWM to prevent jitter.  It also allows reliable infrared control on
projects that include other code with high interrupt latency (such as WS28xxx LED strips)

## Basic usage:
### General flow

* Create an instance of the class.
* Call ```begin()```
* No further action is needed for the class to receive incoming event messages.
* Poll for incoming events using ```read()```, which always returns immediately, with a return value of 0 and a bit count of 0 if there is no message received.

#### Notes
* An event is the arrival of a complete message (such as a complete IR command, or a complete RFID card swipe, or a complete PWM timing measurement).  You can also detect a received event without removing it from the buffer, by checking ```capturedBitCount``` for a non-zero value.
* Each instance will only buffer one complete "message" event.  If a second event arrives before the first event is read, the first event is overwritten and discarded.  This design is intentional.

### Infrared receiver:

Declare an instance of PulseCapture, providing the desired pin number, and 'I' for infrared.
```
  PulseCapture IR_receiver(8, 'I');
```
In your ```setup()```:
```
  IR_receiver.begin();
```
In your ```loop()```, you can poll for received IR messages with the ```read()``` function.  A non-zero return value indicates a message received.

```  
  byte receivedBitCount;
  unsigned long message =  IR_receiver.read(&receivedBitCount);
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

Wiegand is a popular protocol for RFID readers.  Most readers on the market will return messages of
26 or 34 bits (which represent 24- and 32-bit ID numbers with parity bits).  This library removes the
parity bits and only returns the 24- or 32-bit ID.

Additionally, an RFID reader with a numeric keypad might report keypresses in the form of 4-bit messages.
The messages will be the numbers 0 thru F (hex).

The PulseCapture library provides a derived Wiegand class to simplify the creation of PulseCapture on two
simultaneous pins.  Create it as follows:

```
  Wiegand wiegand_receiver(4, 5);   // Read Wiegand protocol on pins 4 (Data0) and 5 (Data1)
```
In your ```loop()```, receivedBitCount will indicate 4, 24, or 32 when a message is received, or 0 if none.

```  
  byte receivedBitCount;
  unsigned long message =  Wiegand.read(&receivedBitCount);
  if (receivedBitCount) {
    Serial.print(receivedBitCount);
    Serial.print("-bit message received: ");
    Serial.println(message);
  }
```

## System resources impacted

This library requires complete control of Timer1 in order to work.  Because of that, using PWM as follows will conflict with this library:

* Pins 9 and 10 (Uno and Nano only)
* Pins 11 and 12 (Mega only)
* Pins 6,7,8 while capturing on pin 49 (Mega only)
* Pins 44,45,46 while capturing on pin 48 (Mega only)

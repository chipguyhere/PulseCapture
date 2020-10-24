# PulseCapture

PulseCapture is an interrupt-driven library for Arduino Uno/Nano/Mega that captures several types of pulsed digital signals:

* Infrared (the most common NEC protocol)
* Wiegand (two-wire protocol used in RFID readers)
* Servo PWM with 0.5μs resolution

On Uno and Nano, you can use any digital or analog input.  The Pin Change Interrupt is used for capture, which is hardware-supported on all pins.

On Arduino Mega, these pins have hardware interrupt support and are the only ones supported by the library: 10, 11, 12, 13, 48, 49, 50, 51, 52, 53, and A8 thru A15.

This library uses an enhanced Timer Input Capture hardware feature when your input is connected to the specific pin(s) that support it.  This is highly recommended for infrared and servo PWM input, especially on projects that have excessive interrupt latency elsewhere (such as those driving WS281xx LED strips).  Timer input capture provides an enhanced resolution of 0.5μs and eliminates servo jitter.

* On Uno and Nano, pin 8
* On Mega, pins 48 and 49.


## Basic usage:
### General flow

* Create an instance of the class.
* Call ```begin()```
* No further action is needed for the class to receive incoming event messages.
* Poll for incoming events using ```read()```, which always returns immediately, with a return value of 0 and a bit count of 0 if there is no message received.

#### Notes
* An event is the arrival of a complete message (such as a complete IR command, or a complete RFID card swipe, or a complete PWM timing measurement).
* You can detect a received event without removing it from the buffer by checking ```capturedBitCount``` for a non-zero value.
* Each instance will only buffer one complete "message" event.  If a second event arrives before the first event is read, the first event is overwritten and discarded.  This design is intentional.

### Infrared receiver:

Declare an instance of PulseCapture, providing the desired pin number, and 'I' as the protocol identifier for infrared.
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

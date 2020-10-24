# PulseCapture

PulseCapture is an interrupt-driven library for Arduino Uno/Nano/Mega that captures several types of timed digital signals:

* Infrared (the most common NEC protocol)
* Wiegand (two-wire protocol used in RFID readers)
* Servo PWM with 0.5μs resolution)

On Uno and Nano, pulses can be captured on any digital or analog input (not just pins 2 and 3), as the library uses Pin Change Interrupts to
capture pulses.  On Mega, this library supports capture only on these hardware-supported pins: 10, 11, 12, 13, 48, 49, 50, 51, 52, 53, and A8 thru A15.

This library uses hardware-based Timer Input Capture when your input is connected to the specific pin(s) that support it.
These provide the highest resolution and are the best pins to use with this library.  On Uno and Nano, that's pin 8.  On Mega, that's pins 48 and 49.

Hardware input capture uses a 16-bit timer to timestamp the incoming pulses when they arrive, providing a resolution of 0.5μs that is unaffected by other interrupts running
on the microcontroller.  This is especially important when receiving servo PWM to prevent jitter.  It also allows reliable infrared control on
projects that include other code with high interrupt latency (such as WS28xxx LED strips)

## Basic usage:
### Infrared receiver:

Declare an instance of PulseCapture, providing the desired pin number, and 'I' for infrared.
```
  PulseCapture IR_receiver(8, 'I');
```
In your ```setup()```:
```
  IR_receiver.begin();
```
In your ```loop()```, you can poll for received IR messages as follows:
```  
  byte receivedBitCount;
  unsigned long message =  IR_receiver.read(&receivedBitCount);
  if (receivedBitCount==32) {
    Serial.println(message, HEX);
  } else if (receivedBitCount==1) {
    // key-held pulse is delivered as a series of single-bit message of "1"
    // as long as the key remains held down
    Serial.println("(key held down)");    
  }
```


## System resources impacted

This library requires complete control of Timer1 in order to work.  Because of that, using PWM as follows will conflict with this library:

* Pins 9 and 10 (Uno and Nano only)
* Pins 11 and 12 (Mega only)
* Pins 6,7,8 while capturing on pin 49 (Mega only)
* Pins 44,45,46 while capturing on pin 48 (Mega only)

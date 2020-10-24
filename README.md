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


### Wiegand (RFID) receiver:

Wiegand is a popular two-wire protocol for RFID readers.  Most readers that use this protocol will return messages of
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

Just like usual, in ```setup()``` there needs to be ```wiegand.begin();```

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

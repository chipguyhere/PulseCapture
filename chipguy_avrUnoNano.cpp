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


#include <Arduino.h>
#include "chipguy_pulsecapture.h"


#if defined (ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_UNO)
volatile uint16_t ovfcount=0;


ISR(TIMER0_COMPB_vect) {
	uint16_t ovf = ovfcount;
	if (TIFR1 & _BV(TOV1)) {
		ovf++;
	}
	cgh_pulsecapture_add_tick_to_queue(ovf*65536 + TCNT1);  
}






int PulseCapture::begin(void) {
  noInterrupts();
  pinMode(pin, INPUT);


  if (pulsecapture_began==false) {
    pulsecapture_began=true;
    
    // Set timer1 to normal counting, all the way up to 0xFFFF
    // WGM13,12,11,10 = 0
    TCCR1A &= ~(_BV(WGM11)|_BV(WGM10));
    TCCR1B &= ~(_BV(WGM13)|_BV(WGM12));
  
    // Set input capture edge detection to falling (we will flip this as we get edges)
    TCCR1B &= ~(_BV(ICES1));
  
    // Set prescaler to /64, so we're counting at 250KHz in units of 4ms
    TCCR1B &= ~(_BV(CS12));
    TCCR1B |= _BV(CS11)|_BV(CS10);
  
    // Turn on timer overflow interrupt
    TIMSK1 |= _BV(TOIE1);
    
    // Turn on timer 0 compare B for our tick
    TIMSK0 |= _BV(OCIE0B);  


  }

  bool use_hardware_capture=false;

  if (pin==8) use_hardware_capture=true;
  if (protocol == 'W' || protocol == '0') use_hardware_capture=false;

  if (use_hardware_capture) {     

    portid = '1';
    mask=1;
    lastRead = 1;
    
    TIMSK1 |=  _BV(ICIE1); // Turn on Timer1 input capture interrupt for pin 8

  } else {
  
  	
    portid = ((digitalPinToPort(pin) - 1) % 16) + 'A';
    mask = digitalPinToBitMask(pin);
    lastRead = mask;

    if (digitalPinToPCICR(pin)==0) {
    	// shouldn't apply.  All valid pins have a PCICR      
    } else {    
      // turn on pin change interrupts for the pin
      *digitalPinToPCMSK(pin) |= _BV(digitalPinToPCMSKbit(pin));
      *digitalPinToPCICR(pin) |= _BV(digitalPinToPCICRbit(pin));
    }
    
  }  



  interrupts();


  return 0;
}



void ISRX(uint8_t PINx, char _portid, uint16_t cnt) {
  uint16_t ovfs = ovfcount;
  if (cnt < 0x8000 && (TIFR1 & _BV(TOV1))) ovfs++;
  uint32_t timestamp = ovfs;
  timestamp = timestamp * 65536;
  timestamp = timestamp + cnt;
  
  cgh_pulsecapture_add_event_to_queue(PINx, _portid, timestamp);

    
}

ISR(PCINT0_vect) { ISRX(PINB, 'B', TCNT1); }
ISR(PCINT1_vect) { ISRX(PINC, 'C', TCNT1); }
ISR(PCINT2_vect) { ISRX(PIND, 'D', TCNT1); }


void finish_capture_isr(char portid, uint8_t gotfall, uint16_t icr, uint16_t tcnt) {

	uint16_t ovf = ovfcount;
	if (icr >= 0xc000 && tcnt < 0x4000) ovf++;
	if (icr < 0x4000 && tcnt >= 0xc000) ovf--;
	
	uint32_t timestamp = ovf;
	timestamp *= 65536;
	timestamp += icr;
	cgh_pulsecapture_add_event_to_queue(gotfall ? 0 : 1, portid, timestamp);
  
}

ISR(TIMER1_CAPT_vect) {
 
 	uint16_t icr = ICR1;
 	uint16_t tcnt = TCNT1;

  TCCR1B ^= _BV(ICES1); // alternate direction of edge of next capture  

  // did we capture a rise or fall?
  uint8_t gotfall =   TCCR1B & _BV(ICES1);
  finish_capture_isr('1', gotfall,icr,tcnt);
}

ISR(TIMER1_OVF_vect) {
  ovfcount++; 
}




#endif // #if defined (ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_UNO)

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
#include "chipguy_PulseCapture.h"
#include "chipguy_PulseCapture_privates.h"



#if defined (ARDUINO_AVR_NANO) || defined(ARDUINO_AVR_UNO)
#define TIMERx 1
#define ICRx ICR1
#define TCNTx TCNT1
#define TIFRx TIFR1
#define TCCRxA TCCR1A
#define TCCRxB TCCR1B
#define TIMSKx TIMSK1
#define HWPINx 8
#define TIMERx_CAPT_vect TIMER1_CAPT_vect
#define TIMERx_OVF_vect TIMER1_OVF_vect
void ISRX(uint8_t PINx, char _portid, uint16_t cntx);
ISR(PCINT0_vect) { ISRX(PINB, 'B', TCNTx); }
ISR(PCINT1_vect) { ISRX(PINC, 'C', TCNTx); }
ISR(PCINT2_vect) { ISRX(PIND, 'D', TCNTx); }




volatile uint16_t ovfcount=0;
volatile byte mainTimer=TIMERx;
bool timerx_is_servo=false;

ISR(TIMER0_COMPB_vect) {
	if (mainTimer==0) {
		cgh_pulsecapture_add_tick_to_queue(micros());  	
	} else {
		uint16_t ovf = ovfcount;
		if (mainTimer==TIMERx) { 
			if (TIFRx & _BV(TOV1)) ovf++;		
			cgh_pulsecapture_add_tick_to_queue(ovf*65536 + TCNTx);  		
		}
	}
}



int PulseCapture::begin(void) {

	int rv=0;

  noInterrupts();
  pinMode(pin, INPUT);
	

  if (pin==HWPINx) { // Using hardware capture
    portid = '0' + TIMERx;
    mask=1;
    lastRead = digitalRead(pin)==HIGH ? 1 : 0;
    
		if (pin==HWPINx) TIMSKx |= _BV(ICIE1); 
		if (protocol=='P') {
			if (pin==HWPINx) timerx_is_servo=true;
			clockrate=21;
			rv=3;
		}  
     
  } else {
  
    portid = ((digitalPinToPort(pin) - 1) % 16) + 'A';
    mask = digitalPinToBitMask(pin);
    lastRead = mask;

    if (digitalPinToPCICR(pin)==0) {
    	return -1;      
    } else {    
      // turn on pin change interrupts for the pin
      *digitalPinToPCMSK(pin) |= _BV(digitalPinToPCMSKbit(pin));
      *digitalPinToPCICR(pin) |= _BV(digitalPinToPCICRbit(pin));
      rv=2;
    }
    
  }  


  if (pulsecapture_began==false) {
    pulsecapture_began=true;
	}

	// see comments for TCCR1x to expand the meaning of this.

	// WGM13,12,11,10 = 0
	TCCRxA &= ~(_BV(WGM11)|_BV(WGM10));
	TCCRxB &= ~(_BV(WGM13)|_BV(WGM12));
	
	// Set input capture edge detection to falling (we will flip this as we get edges)
	
	TCCRxB &= ~(_BV(ICES1));

	// If servo
	// Set prescaler to /64, so we're counting at 2MHz
	// else
	// Set prescaler to /64, so we're counting at 250KHz in units of 4ms
	if (timerx_is_servo) {
		TCCRxB &= ~(_BV(CS12)|_BV(CS10));
		TCCRxB |= _BV(CS11);
	} else {
		TCCRxB &= ~(_BV(CS12));
		TCCRxB |= _BV(CS11)|_BV(CS10);
	}
	// Turn on timer overflow interrupt
	TIMSKx |= _BV(TOIE1);
	
	if (timerx_is_servo==false) mainTimer=TIMERx;
	else mainTimer=0,main_timer_is_micros=true;
	
	// Turn on timer 0 compare B for our tick
	TIMSK0 |= _BV(OCIE0B);  
	
	// synchronize all timers
	TCNTx=0;
		
  interrupts();

  return rv;
}




void ISRX(uint8_t PINx, char _portid, uint16_t cntx) {
	if (mainTimer==0) {
		cgh_pulsecapture_add_event_to_queue(PINx, _portid, micros());
		return;
	}

  uint16_t ovfs = ovfcount;
	if (mainTimer==TIMERx && TIFRx & _BV(TOV1)) ovfs++;
  uint32_t timestamp = ovfs;
  timestamp = timestamp * 65536;
  if (mainTimer==TIMERx) timestamp = timestamp + cntx;
  
  cgh_pulsecapture_add_event_to_queue(PINx, _portid, timestamp);

    
}



void finish_capture_isr(char portid, uint8_t gotfall, uint16_t icr, uint16_t tcnt) {

	uint16_t ovf = ovfcount;
	if (icr >= 0xc000 && tcnt < 0x4000) ovf++;
	if (icr < 0x4000 && tcnt >= 0xc000) ovf--;
	
	uint32_t timestamp = ovf;
	timestamp *= 65536;
	timestamp += icr;
	cgh_pulsecapture_add_event_to_queue(gotfall ? 0 : 1, portid, timestamp);
  
}

ISR(TIMERx_CAPT_vect) {
 
 	uint16_t icr = ICRx;
 	uint16_t tcnt = TCNTx;

  TCCRxB ^= _BV(ICES1); // alternate direction of edge of next capture  

  // did we capture a rise or fall?
  uint8_t gotfall =   TCCRxB & _BV(ICES1);
  finish_capture_isr('0'+TIMERx, gotfall,icr,tcnt);
}


ISR(TIMERx_OVF_vect) {
  if (mainTimer==TIMERx) ovfcount++;
}

#endif






































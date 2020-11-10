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
#include "chipguy_pulsecapture_privates.h"


#ifdef ARDUINO_AVR_NANO_EVERY


// Quick refresher on Arduino Nano Every / atmega4809 timers.
// There are five 16-bit timers: TCA0, TCB0, TCB1, TCB2, TCB3
// TCA0 has special features that the TCB timers depend on.
// TCA0 contains the prescaler that feeds into the TCB clocks.
//   (without the prescaler, /1 and /2 are the only options).
// TCA0 has three compare channels (CMP0,CMP1,CMP2) that drive compare interrupts and PWM.
// Arduino sets TCA to count from 0 to 0xFF and rolls over 1024 times per second.

// TCB0,1,2,3 can each capture one thing: an edge, a pulse, or an overflow.
// TCB3 is driven by TCA0 and is used by millis() to capture overflows
// TCB0,1,2 are available by default.  All of them can capture input "channels"
// TCB0,1 might be used by PWM.
// TCB1 might be used by tone();




// We'll use timer TCA0 comparator 1 to look for overflows on TCBx
// (because we can't do captures and overflows at the same time)
volatile uint16_t ovfwatch = 0;
volatile uint16_t ovfcount=0;

void isr_tcbx(TCB_t &TCBx, char portid);

// Taking timers B0,B1,B2 for hardware capture feature (leaving B3 alone)
// One timer is used per active pin (except Wiegand, which doesn't use the timers)
// TODO: MAKE IT EASIER FOR SKETCH WRITERS TO NOT TAKE 3 TIMERS IF WE DON'T NEED 3
// but for now, to stop this library from taking a timer, just comment out the ISR,
// and set the timerAvailable flag to false (similar to Timer B3)
bool timerAvailable[4] = {true,true,true,false};
ISR(TCB0_INT_vect) { isr_tcbx(TCB0, '0'); }
ISR(TCB1_INT_vect) { isr_tcbx(TCB1, '1'); }
ISR(TCB2_INT_vect) { isr_tcbx(TCB2, '2'); }
//ISR(TCB3_INT_vect) { isr_tcbx(TCB3, '3'); }


TCB_t *mainTimer=NULL;

ISR(TCA0_CMP1_vect) {
  TCA0.SINGLE.INTFLAGS |= 0x20; // acknowledge the interrupt
  if (mainTimer) {  
    uint16_t cnt = mainTimer->CNT;
    if (cnt < ovfwatch) {
      ovfcount++;
      ovfwatch=cnt;
    } else {
      ovfwatch=cnt;
    }
    cgh_pulsecapture_add_tick_to_queue(ovfcount*65536 + cnt);  
    
  } else {
    cgh_pulsecapture_add_tick_to_queue(micros());    
  }
  
  // TODO: add timer tick to queue outside of overflow.
  // tick always. 
}


// Get a 32 bit timestamp from a 16 bit time, inferring the
// top 32 bits via ovfcount and ovfwatch
uint32_t cgh_pulse_get_adjusted_timestamp(uint16_t time16) {
  uint16_t ovfcount_copy = ovfcount;
  if (time16 >= 0xc000 && ovfwatch < 0x4000) ovfcount_copy--;
  if (time16 < 0x4000 && ovfwatch >= 0xc000) ovfcount_copy++;
  uint32_t timestamp = ovfcount_copy * 65536UL + time16;
  return timestamp;
}

void isr_tcbx(TCB_t &TCBx, char portid) {
	uint32_t timestamp;
	if ((TCBx.CTRLA & 0x06)==TCB_CLKSEL_CLKDIV2_gc) timestamp = TCBx.CCMP;
	else timestamp = cgh_pulse_get_adjusted_timestamp(TCBx.CCMP);
  byte edge = 1; // rising
  if (TCBx.EVCTRL & 0x10) edge=0;
  TCBx.EVCTRL ^= 0x10;
  cgh_pulsecapture_add_event_to_queue(edge, portid, timestamp);
}


// ISR for when we're using traditional attachedInterrupt and no hardware capture.
void attachedISR(void *param) {
  uint32_t timestamp;
  if (mainTimer) timestamp = cgh_pulse_get_adjusted_timestamp(mainTimer->CNT);
  else timestamp=micros();
  PulseCapture *pc = (PulseCapture*)param;
  //Serial.print(pc->protocol);
  byte pinValue = digitalRead(pc->pin)==HIGH ? 1 : 0;
  cgh_pulsecapture_add_event_to_queue(pinValue, pc->pin, timestamp);
}

void cgh_syncClocks(bool sync0, bool sync1, bool sync2, bool sync3) {
  // assume caller has noInterrupts already called

    // Set up things that all instances of class will use.
    // Goal: sync TCA0 and TCB2.
    // When achieved, they'll count in sync, except, TCA0 goes to 0xFF and TCB2 goes to 0xFFFF
    
    // turn on sync update (restart this timer in sync with TCA0's next restart)
    // turn on clock select of system prescaler (same as TCA0 so they'll count in sync, just we're 16 bits)
    // turn on enable timer
    // TCA0 = 250KHz and we're matching it.

  if (sync0) TCB0.CTRLA = TCB_SYNCUPD_bm + TCB_CLKSEL_CLKTCA_gc + TCB_ENABLE_bm;
  if (sync1) TCB1.CTRLA = TCB_SYNCUPD_bm + TCB_CLKSEL_CLKTCA_gc + TCB_ENABLE_bm;
  if (sync2) TCB2.CTRLA = TCB_SYNCUPD_bm + TCB_CLKSEL_CLKTCA_gc + TCB_ENABLE_bm;
  if (sync3) TCB3.CTRLA = TCB_SYNCUPD_bm + TCB_CLKSEL_CLKTCA_gc + TCB_ENABLE_bm;

  // wait for TCA0 count to roll over, after which, the clocks are in sync
  uint16_t lastTCA0 = TCA0.SINGLE.CNT;
  while (lastTCA0 < TCA0.SINGLE.CNT) lastTCA0 = TCA0.SINGLE.CNT;
  
  // now it's in sync.  turn off SYNCUPD
  mainTimer=0;
  if (sync0) TCB0.CTRLA &= ~TCB_SYNCUPD_bm, mainTimer=&TCB0;
  if (sync1) TCB1.CTRLA &= ~TCB_SYNCUPD_bm, mainTimer=&TCB1;
  if (sync2) TCB2.CTRLA &= ~TCB_SYNCUPD_bm, mainTimer=&TCB2;
  if (sync3) TCB3.CTRLA &= ~TCB_SYNCUPD_bm, mainTimer=&TCB3;
	if (mainTimer==0) main_timer_is_micros=true;
  
}


int PulseCapture::begin(void) {
	int rv=0;
	
  // Interrupts need to be off when accessing 16-bit registers
  pinMode(pin, INPUT_PULLUP);
  portid = 'A' + digitalPinToPort(pin); 
  mask = digitalPinToBitMask(pin);  

  noInterrupts();


  if (pulsecapture_began==false) {
    pulsecapture_began=true;

    cgh_syncClocks(timerAvailable[0], timerAvailable[1], timerAvailable[2], timerAvailable[3]);


    
    // enable TCA compare interrupt
    TCA0.SINGLE.INTCTRL |= TCA_SINGLE_CMP1EN_bm;

  }
  

  byte pos = digital_pin_to_bit_position[pin];
  
  // can we enable hardware capture?
  // TODO: avoid looking for hardware capture if we are trying to avoid using it.
  

  byte selectedChannel=0xff;
  // CHANNELx needs to be set between 0x40 and 0x4F depending on
  // which pin we'll be capturing from.
  byte chanx = pos + 0x40;
  
  byte selectedTimer = 0xff;
    
  if (timerAvailable[3] && EVSYS.USERTCB3==0) selectedTimer=3;
  else if (timerAvailable[2] && EVSYS.USERTCB2==0) selectedTimer=2;
  else if (timerAvailable[1] && EVSYS.USERTCB1==0) selectedTimer=1;
  else if (timerAvailable[0] && EVSYS.USERTCB0==0) selectedTimer=0;
  
  // Avoid taking timers for Wiegand protocol because they're not useful.
  // Instead we'll attachInterrupt and filter for falls.
  if (protocol=='W' || protocol=='0') selectedTimer=0xff;

  if (selectedTimer != 0xff){
    if (portid=='B' || portid=='D' || portid=='F') chanx += 8;
    if (portid=='A' || portid=='B') {
      if (EVSYS.CHANNEL0==0) selectedChannel=0,EVSYS.CHANNEL0=chanx;
      else if (EVSYS.CHANNEL1==0) selectedChannel=1,EVSYS.CHANNEL1=chanx;
    } else if (portid=='C' || portid=='D') {
        if (EVSYS.CHANNEL2==0) selectedChannel=2,EVSYS.CHANNEL2=chanx;
        else if (EVSYS.CHANNEL3==0) selectedChannel=3,EVSYS.CHANNEL3=chanx;
    } else if (portid=='E' || portid=='F') {
        if (EVSYS.CHANNEL4==0) selectedChannel=4,EVSYS.CHANNEL4=chanx;
        else if (EVSYS.CHANNEL5==0) selectedChannel=5,EVSYS.CHANNEL5=chanx;
    }
    // did we find an available channel?
    // if so, route it to TCB2.  (the +1 is required per the datasheet bc 0=disabled)
    if (selectedChannel != 0xff) {
    	rv=3;
      portid = '0' + selectedTimer;
      lastRead = mask = 1;
      selectedChannel++;
      TCB_t *TCBx = &TCB0;
      switch (selectedTimer) {
        case 0: EVSYS.USERTCB0 = selectedChannel; break;
        case 1: EVSYS.USERTCB1 = selectedChannel; TCBx = &TCB1; break;
        case 2: EVSYS.USERTCB2 = selectedChannel; TCBx = &TCB2; break;
        case 3: EVSYS.USERTCB3 = selectedChannel; TCBx = &TCB3; break;
      }
		
			// set timer counter mode to capture event  
			TCBx->CTRLB = TCB_CNTMODE_CAPT_gc;

			// turn on "catch a falling edge" (otherwise catches a rising edge)
			// turn on event input enable.
			TCBx->EVCTRL = TCB_EDGE_bm + TCB_CAPTEI_bm;
	
			// turn on the interrupt
			TCBx->INTCTRL=TCB_CAPT_bm;
		
			// if it's for a servo, let's pull it from available and upclock it to 8MHz
			if (protocol=='P') {
				timerAvailable[selectedTimer]=false;
				TCBx->CTRLA = TCB_CLKSEL_CLKDIV2_gc + TCB_ENABLE_bm;
				//TCBx->CTRLB = TCB_CNTMODE_PW_gc;				
				clockrate=22;      				
				mainTimer=0;
				if (timerAvailable[0]) mainTimer=&TCB0;
				if (timerAvailable[1]) mainTimer=&TCB1;
				if (timerAvailable[2]) mainTimer=&TCB2;
				if (timerAvailable[3]) mainTimer=&TCB3;
				if (mainTimer==0) main_timer_is_micros=true;
			}
		}
  }
  
  if (selectedChannel==0xff) {
    // We were not able to set up hardware capture.
    // Set up a normal pin change detection through attachInterrupt.
    attachInterruptParam(digitalPinToInterrupt(pin), attachedISR, CHANGE, this);
    mask = 1;
    lastRead = digitalRead(pin)==HIGH ? mask : 0;
    portid = pin;
    rv=1;
  }


  
  
  interrupts();
  return rv;
  
}






#endif // #ifdef ARDUINO_AVR_NANO_EVERY

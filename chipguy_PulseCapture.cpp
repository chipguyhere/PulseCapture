/*
PulseCapture by chipguyhere
Copyright 2019-2020, License: GPLv3

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


// Pins Supported:
// Uno/Nano: all digital pins, all analog pins thru A5
//  Hardware Capture: Pin 8 (recommended for capturing IR, serial, and PWM inputs)
// Mega: 10,11,12,13,48,49,50,51,52,53,A8-A15
//  Hardware Capture: Pin 48,49

// Hardware capture means the timestamp is being made by the hardware timer itself, giving
// microsecond-level precision on the timing and more tolerance for interrupt latency.




// Represents the facts of an interrupt we received.
// Goes in a circular buffer for handling with interrupts re-enabled.
struct edgeevent {
  char portid;      
  uint8_t portRead;     // inputs of the port register when we read it (or 0/1 for ICP fall/rise)
  uint32_t timer;
}; 

bool pulsecapture_began=false;
bool main_timer_is_micros=false;
volatile PulseCapture *first_pulsecapture_instance=NULL;

// edgeEventCount must be a power of two.
// suggestions: 8 = normal usage
// 128 = debugging this code with lots of serial print
#define edgeEventCount 32

volatile struct edgeevent edgeEvents[edgeEventCount];
volatile uint8_t edgeEventHead=0;
volatile uint8_t edgeEventTail=0;

void handle_irq_queue();


void debugEdgeEvents() {
	return;

	static uint8_t debugTail = 0;
	
	while (debugTail != edgeEventHead) {
		edgeevent *ee = &edgeEvents[debugTail];
		if (ee->portid != 'T') {
			Serial.print("portid=");
			Serial.print(ee->portid);
			Serial.print(" portRead=");
			Serial.print(ee->portRead);
			Serial.print(" timer=");
			Serial.println(ee->timer);
		}		
		debugTail++;
		if (debugTail==edgeEventCount) debugTail=0;	
	}
	// Before uncommenting this,
	// make sure this is the only caller to handle_irq_queue
	// (comment out other path(s))
	//handle_irq_queue();


}


PulseCapture::PulseCapture() {}

PulseCapture::PulseCapture(byte pin, byte _protocol) {
  init(pin,_protocol);
}

void PulseCapture::init(byte _pin, byte _protocol) {

	// init gets called by the constructor
	// which could happen before setup().
	// interrupts should not be activated until calling begin

	pin=_pin;
	protocol=_protocol;
	
	noInterrupts();
	if (first_pulsecapture_instance==NULL) {
		first_pulsecapture_instance = this;
	} else {
		PulseCapture *pc = first_pulsecapture_instance;
		while (pc->next_pulsecapture_instance) pc=pc->next_pulsecapture_instance;
		pc->next_pulsecapture_instance=this;
	}
	interrupts();
}




uint32_t PulseCapture::read(int &bitcount) {
  bitcount = capturedBitCount;
  if (capturedBitCount==0) return 0;
  uint32_t rv = capturedMessage;
  capturedBitCount=0;
  capturedMessage=0;  
  return rv;
}
uint32_t PulseCapture::read() {
  if (capturedBitCount==0) return 0;
  uint32_t rv = capturedMessage;
  capturedBitCount=0;
  capturedMessage=0;  
  return rv;
}


void cgh_pulsecapture_add_tick_to_queue(uint32_t timestamp) {
/*
  if (edgeEventHead != edgeEventTail) {
    // if the latest entry is a timer tick that has not yet been seen,
    // overwrite it.  "Not been seen" means the circular buffer must
    // contain at least 2 entries: an earlier one that might be getting looked at,
    // plus a later one we might overwrite
    if (((edgeEventHead - edgeEventTail) & (edgeEventCount-1)) >= 2) {
      if (edgeEvents[edgeEventHead].portid=='T') {
        edgeEventHead = (edgeEventHead-1) & (edgeEventCount-1);         
      }
    }    
  }*/
  // add ticks only while the queue is empty
  if (edgeEventHead == edgeEventTail) cgh_pulsecapture_add_event_to_queue(0, 'T', timestamp);
  
}

void cgh_pulsecapture_add_event_to_queue(uint8_t portRead, char portid, uint32_t timestamp) {
  uint8_t newhead = (edgeEventHead+1) & (edgeEventCount-1);
  if (newhead == edgeEventTail) return;
  struct edgeevent* ee = &edgeEvents[newhead];

  ee->portid = portid;
  ee->portRead = portRead;
  ee->timer=timestamp;
  edgeEventHead = newhead;
  handle_irq_queue();
}


volatile uint8_t in_ISRX=0;
void handle_irq_queue() {
  if (in_ISRX) return;
  in_ISRX=1;  
  interrupts();
  byte processed=0;

  while (edgeEventHead != edgeEventTail) {
    byte newtail = (edgeEventTail + 1) & (edgeEventCount-1);
    struct edgeevent *ee = &edgeEvents[newtail];
    
    for (PulseCapture *ei = first_pulsecapture_instance; ei != NULL; ) {
      ei = (PulseCapture*)(ei->_handle_irq(ee));  
    }
    edgeEventTail = newtail; 
    if (++processed==32) break;
  }

  in_ISRX=0;
  
}


void PulseCapture::_handle_edge(char edgeKind, uint32_t timediff32, uint16_t timediff) {

	
}

void* PulseCapture::_handle_irq(void *eev) {
  struct edgeevent *ee = (struct edgeevent*)eev;
    
	char edgeKind = 'T';  // T=timer R=rise F=fall
  
  
  // Shortcut: if the interrupt is a timer tick, but we're not in a word, then we don't care.      
  if (ee->portid=='T' && inword==0) {
    // don't care.  Timer ticks can only end messages, not begin them.
  } else if (ee->portid != 'T' && ee->portid != portid) {
    // if it's for a different instance we also don't care.        
  } else if (ee->portid != 'T' && (ee->portRead & mask) == lastRead) {
    // if it's for our port, but there's no change to our bits, then also don't care.
    // (this case gets tripped if a pulse is so short that it ended before we could read it)        
  } else if (protocol=='0' && ee->portid=='T') {
    // no need to give timer to both Wiegand pins otherwise we're timering twice, so, don't care        
  } else {
    if (ee->portid != 'T') {
      lastRead = ee->portRead & mask;         
      if (lastRead) edgeKind = 'R';       
      else edgeKind = 'F';
    }
    
    uint32_t rcvtime = ee->timer;
		long timediff32 = rcvtime - lastTimestamp;
		if (edgeKind != 'T') lastTimestamp = rcvtime;		
		switch (clockrate) {
			case 0: if (main_timer_is_micros) break; // micros is already in us
					timediff32 = timediff32 * 4; break; // ticks are in units of 4us	
			case 20: timediff32 = timediff32 * 4000; break; // ticks are 4us, want ns
			case 21: timediff32 = timediff32 * 500; break; // ticks are 0.5us, want ns
			case 22: while (timediff32 < 0) timediff32 += 65536;
				timediff32 = timediff32 * 125; break; // ticks are 125ns and not a diff
		}
		uint16_t timediff = (uint32_t)timediff32;
		if (timediff32 > 65535) timediff=65535;
	
	  _handle_edge(edgeKind,timediff32, timediff);
	  
    /*
    Serial.print("inword now ");
    Serial.println(inword);
    */
  }  
  return next_pulsecapture_instance;
}



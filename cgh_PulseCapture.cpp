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

// Hardware dependency: this module takes over Timer1 on ATmega328p, and also Timer4,5 on ATmega2560



#include <Arduino.h>
#include "cgh_pulsecapture.h"

// Pins Supported:
// Uno/Nano: all digital pins, all analog pins thru A5
//  Hardware Capture: Pin 8 (recommended for capturing IR, serial, and PWM inputs)
// Mega: 10,11,12,13,48,49,50,51,52,53,A8-A15
//  Hardware Capture: Pin 48,49

// Hardware capture means the timestamp is being made by the hardware timer itself, giving
// microsecond-level precision on the timing and more tolerance for interrupt latency.



union uint32x_t {
  uint8_t u8[4];
  uint16_t u16[2];
  uint32_t u32;
};

// Represents the facts of an interrupt we received.
// Goes in a circular buffer for handling with interrupts re-enabled.
struct edgeevent {
  uint8_t *portaddr;    // identifies the interrupt, or 0 for timer overflow
  uint8_t portRead;     // inputs of the port register when we read it (or 0/1 for ICP fall/rise)
  uint32x_t timer;
}; 

bool pulsecapture_began=false;
volatile PulseCapture *first_pulsecapture_instance=NULL;

// edgeEventCount must be a power of two.
#define edgeEventCount 8

volatile struct edgeevent edgeEvents[edgeEventCount];
volatile uint8_t edgeEventHead=0;
volatile uint8_t edgeEventTail=0;
volatile uint16_t ovfcount=0;


Wiegand::Wiegand(byte pinD0, byte pinD1) {
	d0_pulsecapture_instance.init(pinD0, '0');
	init(pinD1, 'W');
}

PulseCapture::PulseCapture() {}

PulseCapture::PulseCapture(byte pin, byte _protocol) {
  init(pin,_protocol);
}

void PulseCapture::init(byte pin, byte _protocol) {

  bool use_hardware_capture=false;

#if defined(model_UNO_NANO)
  if (pin==8 && _protocol != 'W' && _protocol != '0') use_hardware_capture=true;
  // (avoiding enabling hardware capture for Wiegand because no benefit if only for 1 of its 2 signals)
#elif defined (model_MEGA)
  if (pin==48 || pin==49) use_hardware_capture=true;
#endif

  protocol=_protocol;
  
  if (use_hardware_capture) {     

#if defined(model_MEGA) || defined(model_UNO_NANO)
    portaddr = &ICR1L;
    mask=1;
    lastRead = 1;
#endif
    
#if defined(model_UNO_NANO)

    TIMSK1 |=  _BV(ICIE1); // Turn on Timer1 input capture interrupt for pin 8
#elif defined (model_MEGA)
  if (pin==48) TIMSK5 |= _BV(ICIE1); // Pin 48 is capture input for timer5
  if (pin==49) TIMSK4 |= _BV(ICIE1); // Pin 49 is capture input for timer4
#endif    
     
  } else {
  
    portaddr = portInputRegister(digitalPinToPort(pin));
    mask = digitalPinToBitMask(pin);
    lastRead = mask;

#if defined(model_UNO_NANO) || defined(model_MEGA)
    if (digitalPinToPCICR(pin)==0) {
 #ifdef model_MEGA      
      SerialMonitor->print(F("Unable to activate interrupt for pin "));
      SerialMonitor->print(pin);
      SerialMonitor->println(F(".  Edge capture is supported only on pins 10,11,12,13,48,49,50,51,52,53,A8-A15"));
 #endif
      
    } else {    
      // turn on pin change interrupts for the pin
      *digitalPinToPCMSK(pin) |= _BV(digitalPinToPCMSKbit(pin));
      *digitalPinToPCICR(pin) |= _BV(digitalPinToPCICRbit(pin));
    }
#endif
    
  }  

  if (first_pulsecapture_instance==NULL) {
    first_pulsecapture_instance=this;
  } else {
    PulseCapture *ei = first_pulsecapture_instance;
    while (ei->next_pulsecapture_instance != NULL) ei = ei->next_pulsecapture_instance;
    ei->next_pulsecapture_instance = this;
  }

  interrupts();

}


int PulseCapture::begin(void) {
  noInterrupts();

  if (pulsecapture_began==false) {
    pulsecapture_began=true;

#ifdef model_MEGA
    // stop all timers so we can sync them
    GTCCR = (1<<TSM)|(1<<PSRASY)|(1<<PSRSYNC);

    // do to timers 4,5 what we're doing to timer 1.
    // see comments for TCCR1x to expand the meaning of this.
    TCCR4A &= ~(_BV(WGM11)|_BV(WGM10));
    TCCR4B &= ~(_BV(WGM13)|_BV(WGM12));
    TCCR4B &= ~(_BV(ICES1));
    TCCR4B &= ~(_BV(CS12)|_BV(CS10));
    TCCR4B |= _BV(CS11);
    TCCR5A &= ~(_BV(WGM11)|_BV(WGM10));
    TCCR5B &= ~(_BV(WGM13)|_BV(WGM12));
    TCCR5B &= ~(_BV(ICES1));
    TCCR5B &= ~(_BV(CS12)|_BV(CS10));
    TCCR5B |= _BV(CS11);


#endif
#if defined(model_MEGA) || defined(model_UNO_NANO)
    
    // Set timer1 to normal counting, all the way up to 0xFFFF
    // WGM13,12,11,10 = 0
    TCCR1A &= ~(_BV(WGM11)|_BV(WGM10));
    TCCR1B &= ~(_BV(WGM13)|_BV(WGM12));
  
    // Set input capture edge detection to falling (we will flip this as we get edges)
    TCCR1B &= ~(_BV(ICES1));
  
    // Set prescaler to /8, so we're counting at 2MHz in units of 0.5us
    TCCR1B &= ~(_BV(CS12)|_BV(CS10));
    TCCR1B |= _BV(CS11);
  
    // Turn on timer overflow interrupt
    TIMSK1 |= _BV(TOIE1);
#endif
#ifdef model_MEGA
    // synchronize all timers
    TCNT1H=0;
    TCNT1L=0;
    TCNT4H=0;
    TCNT4L=0;
    TCNT5H=0;
    TCNT5L=0;
    // restart all timers
    GTCCR = 0;

#endif

  }

  interrupts();

  return begin_status;
}


uint32_t PulseCapture::read(uint8_t *bitcount) {
  *bitcount = capturedBitCount;
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




const uint16_t bittimes9600[] PROGMEM = {0, 156,260,365,469,573,677,781,885,990};

volatile uint8_t in_ISRX=0;
void handle_irq_queue() {
  if (in_ISRX) return;
  in_ISRX=1;  
  interrupts();

  while (edgeEventHead != edgeEventTail) {
    byte newtail = (edgeEventTail + 1) & (edgeEventCount-1);
    struct edgeevent *ee = &edgeEvents[newtail];
    
    for (PulseCapture *ei = first_pulsecapture_instance; ei != NULL; ) {
      ei = (PulseCapture*)(ei->_handle_irq(ee));  
    }
    edgeEventTail = newtail; 
  }

  in_ISRX=0;
  
}

void* PulseCapture::_handle_irq(void *eev) {
  struct edgeevent *ee = (struct edgeevent*)eev;
    
  
  // Shortcut: if the interrupt is a timer tick, but we're not in a word, then we don't care.      
  if (ee->portaddr==0 && inword==0) {
    // don't care.  Timer ticks can only end messages, not begin them.
  } else if (ee->portaddr != 0 && ee->portaddr != portaddr) {
    // if it's for a different pin on the same port, we also don't care.        
  } else if (ee->portaddr != 0 && (ee->portRead & mask) == lastRead) {
    // if it's for our port, but there's no change to our bits, then also don't care.
    // (this case gets tripped if a pulse is so short that it ended before we could read it)        
  } else if (protocol=='0' && ee->portaddr==0) {
    // no need to give timer to both Wiegand pins otherwise we're timering twice, so, don't care        
  } else {
    bool isTimer=false, isRise=false, isFall=false;
    if (ee->portaddr==0) isTimer=true;
    else {
      lastRead = ee->portRead & mask;         
      if (lastRead) isRise=true;       
      else isFall=true;
    }

    // At the time of this writing, a timer event comes every 32.768ms, originated by timer1
  
    uint32_t rcvtime = ee->timer.u32;
    // using an input with two pins? sync the time stamps to the later.
    if (protocol=='0') 
      if (((long)lastTimestamp - (long)next_pulsecapture_instance->lastTimestamp) > 0) 
        next_pulsecapture_instance->lastTimestamp = lastTimestamp;
      else lastTimestamp = next_pulsecapture_instance->lastTimestamp;

    
    long timediff32 = (long)rcvtime - (long)lastTimestamp;
    timediff32 = timediff32 / 2; //------------- hardware dependency: hardcoded clock rate of 2 ticks per microsecond
    uint16_t timediff = (uint32_t)timediff32;
    if (timediff32 > 65535) timediff=65535;
    // update the time stamp for edge events but not timer events
    // (so future timediffs always refer to time since last edge)
    if (isTimer==false) lastTimestamp = rcvtime;



    if (protocol=='P') {
      // CAPTURE FOR SERVO PWM
      // it's simple: on falls, if it looks like a valid PWM pulse, the message is its length.
      // Valid pulses are nominally 1000-2000us, but overshoots are allowed.
      if (isFall && timediff > 500 && timediff < 2500) {
        capturedMessage = timediff;
      }
   
    } else if (protocol=='I') {
      // CAPTURE FOR IR SIGNALS          
      if (isFall) {            
        if (inword==0 || timediff >= 30000U) {    // first edge of a message?
          inword=1;
          bitsreceived=0;
        } else if (inword==1) { // second edge of a message?  (got the start bit, but we're before the first data bit)
          if (timediff > 2000 && timediff < 2500) {
            if (((long)rcvtime - (long)(capbuf)) < 131072*2) {
              // got the signal that says button is being held down.
              // While inword==0, capbuf is borrowed to be the timestamp of the last good message.
              // for a repeat to be valid, we needed to receive a good message in the last 125ms,
              // and the non-ISR code needs to have picked up the original message to know what to repeat.
              if (capturedBitCount==0) {
                  capturedBitCount=1;
                  capturedMessage=1;
              }
              if (isTimer) {
              
                inword=0;
              }
              capbuf=rcvtime;
            }
          } else
          // expecting about 4450
          if (timediff < 4000 || timediff > 5000) {
            inword=0;
            
          } else {
            inword=2;
            capbuf=0;               
          }
          // while in a word,
          // the amount of time after a fall (i.e. the beginning of an IR pulse, since pulse is low)
          // tells us what bit it is.              
          // expecting around 562.5 for a low bit, or 1687.5 for a high bit.
          // look for anything deviating from that, cancelling our word if so.
          // Saw a remote that had ~19925 in one specific bit position
        } else if (inword != 2) {
          inword=0;
          
        } else if (timediff < 400 || timediff > 1880) { // timediff > 20500 || (timediff > 1830 && timediff < 19500)) {      

          inword=0;
          
        } else if (timediff > 800 && timediff < 1400) {

          inword=0;
        } else if (bitsreceived < 32) {
          if (timediff > 1000) capbuf |= (0x80000000UL >> bitsreceived);        
          bitsreceived++;
          if (bitsreceived==32) {
            capturedMessage = capbuf;
            capbuf = rcvtime;              
            capturedBitCount=32;
            inword=0;           
                 
          }
        }
      } else if (isRise) { // gotrise          
        // normal rises only come with a time diff of:
        // ~9000 for a sync bit
        // ~4500 for a sync bit on some other remote I saw
        // ~600 for a normal pulse
        // rises don't start messages because IR receiver is idle high.
        // rises can be shorter than expected in case of low light/contrast.            
        if (timediff > 11000) inword=0;
        if (inword==2 && (timediff < 300 || timediff > 750)) inword=0;
      } else if (isTimer && timediff > 10000) {
        inword=0;
        
      }
    } else if (protocol=='9') {
      // RS232 Serial 9600bps TTL

      if (inword==0 && isFall) {
        inword=1;
        bitsreceived=0;
        // we will assume all bits are 1 until we receive them as zero, because
        // the last edge could come before the last bit on the timeline, all of
        // which will be 1's in that case.
        charbuf = 0xff;
        
      } else if (inword && isTimer==false) {
        byte newbits;
        for (newbits=1; newbits<=9; newbits++) if (timediff < pgm_read_word_near(bittimes9600+newbits)) break;
        if (newbits==10) inword=0;
        else while (newbits-- && bitsreceived < 9) {
          // if we got a rise, then we just finished getting some zeroes.
          // we already assume everything's a 1 until informed otherwise, which happens here.
          if (bitsreceived && isRise) charbuf &= ~(1 << (bitsreceived-1));              
          bitsreceived++;        
        }
      } else if (inword && isTimer && timediff > 1200) {
        // if we got a timeout condition and last edge was in the middle of a word, then we accept the assumption
        // that the rest of the bits are 1's, and assume all the bits to be received.
        bitsreceived=9;
      }  
              
      if (bitsreceived >= 9) {
        // got a full byte.  look at it on the protocol.
        byte c = charbuf;

        if (incard==0 && c==2) incard++;
        else if (incard==1 && c==0x0a) incard++;
        else if (incard==2 || incard==3) capbuf=0,incard++;
        else if (incard < 4) {
          incard=0;
          return;
        } else if (incard < 8) {
          capbuf = capbuf * 256u;
          capbuf += c;
          incard++;
        } else if (incard==8) incard++;
        else if (incard==9 && c==3) {
          capturedMessage=capbuf;
          capturedBitCount=32;
          incard=0;    
        } else {
          incard=0;
        }

        // if we had a rise, we're out of word (stop bit).  if fall, we're starting a new word.
        inword = isFall ? 1 : 0;
        bitsreceived=0;
        charbuf=0xff;
      }
            
      
    } else if (protocol=='W' || protocol=='0') {
      // if we have a 0, then we are looking at the 0-pin of a two-pin Wiegand setup.
      PulseCapture *wi = this;
      bool is0=false, is1=true;
      if (protocol=='0') is0=true,is1=false,wi=next_pulsecapture_instance;
      
      // On Wiegand, the signal is idle high.  We get bits when pin0 or pin1 falls.
      // As long as its fall isn't "late" (compared to earlier bits), it's good anytime.
      // "A little bit late" -- we ignore the bit.  (between 5 and 65ms)
      // "A lot late" -- consider it a brand new message (65ms+)

      if (isFall) {            
        if (wi->inword==0 || timediff >= 65535) {
          wi->inword=1;
          wi->bitsreceived=1;
          wi->capbuf=is1 ? 1 : 0;              
        } else if (timediff < 5000) {
          if (wi->bitsreceived < 33) {
            // On a 34-bit Wiegand message we only want the middle 32 bits.
            // We will dump the last (34th) bit by not saving it.
            // The first bit will get dumped by capbuf only having room for 32 bits.
            wi->capbuf <<= 1;             
            if (is1) wi->capbuf |= 1;
          }
          wi->bitsreceived++;
        }
      } else if (isTimer && timediff > 5000) {            
        if (wi->bitsreceived==4 || wi->bitsreceived==26 || wi->bitsreceived==34) {
          wi->capturedMessage = wi->capbuf;
          wi->capturedBitCount=4;              
          if (wi->bitsreceived==26) {
            wi->capturedMessage = (wi->capbuf >> 1) & 0xFFFFFF;
            wi->capturedBitCount=24;
          } else if (wi->bitsreceived==34) {
            wi->capturedBitCount=32;                
          }
        }
        wi->bitsreceived=0;
        wi->inword=0;              
                  
      }  
    }
  }  
  return next_pulsecapture_instance;
}


# if defined (model_UNO_NANO) || defined(model_MEGA)

void ISRX(uint8_t PINx, uint8_t *Ix, uint8_t xH, uint8_t xL) {
  uint8_t ccl = xL;
  uint8_t cch = xH;
    
  uint8_t newhead = (edgeEventHead+1) & (edgeEventCount-1);
  if (newhead == edgeEventTail) return;
  struct edgeevent* ee = &edgeEvents[newhead];

  ee->portaddr = Ix;
  ee->portRead = PINx;
  ee->timer.u8[0]=ccl;
  ee->timer.u8[1]=cch;
  ee->timer.u16[1] = ovfcount;
  if (Ix!=0 && cch < 0x80 && (TIFR1 & _BV(TOV1))) ee->timer.u16[1]++;  
  edgeEventHead = newhead;  

  handle_irq_queue();
}
#endif

# if (defined model_UNO_NANO)
ISR(PCINT0_vect) { ISRX(PINB, &PINB, TCNT1H, TCNT1L); }
ISR(PCINT1_vect) { ISRX(PINC, &PINC, TCNT1H, TCNT1L); }
ISR(PCINT2_vect) { ISRX(PIND, &PIND, TCNT1H, TCNT1L); }
# elif (defined model_MEGA)
ISR(PCINT0_vect) { ISRX(PINB, &PINB, TCNT1H, TCNT1L); }
ISR(PCINT2_vect) { ISRX(PINK, &PINK, TCNT1H, TCNT1L); }
#endif

#if defined(model_MEGA) || defined(model_UNO_NANO)

void finish_capture_isr(uint8_t gotfall, uint8_t cl, uint8_t ch, uint8_t cch) {

  uint8_t newhead = (edgeEventHead+1) & (edgeEventCount-1);

  if (newhead !=edgeEventTail) {
    struct edgeevent* ee = &edgeEvents[newhead];
    ee->portaddr = &ICR1L; // TODO: does this break on Mega if multiple instances?
    ee->timer.u16[1]=ovfcount;
    ee->timer.u8[0]=cl;
    ee->timer.u8[1]=ch;  
    // if timer has already gone back to 0 and we've already cleared TOV1,
    // then ovfcount may be too high by 1, fix that
    if ((cch < ch) && !(TIFR1 & _BV(TOV1))) ee->timer.u16[1]--;
    
    ee->portRead = gotfall ? 0 : 1;
    edgeEventHead = newhead;  
  }
  handle_irq_queue();
  
}

ISR(TIMER1_CAPT_vect) {
 
  uint8_t cl = ICR1L;
  uint8_t ch = ICR1H;

  uint8_t ccl = TCNT1L;
  uint8_t cch = TCNT1H;

  TCCR1B ^= _BV(ICES1); // alternate direction of edge of next capture  

  // did we capture a rise or fall?
  uint8_t gotfall =   TCCR1B & _BV(ICES1);
  finish_capture_isr(gotfall,cl,ch,cch);
}
#endif

#ifdef model_MEGA
ISR(TIMER4_CAPT_vect) {
  uint8_t cl = ICR4L;
  uint8_t ch = ICR4H;
  uint8_t ccl = TCNT4L;
  uint8_t cch = TCNT4H;
  TCCR4B ^= _BV(ICES1); // alternate direction of edge of next capture  
  // did we capture a rise or fall?
  uint8_t gotfall =   TCCR4B & _BV(ICES1);
  finish_capture_isr(gotfall,cl,ch,cch);
}
ISR(TIMER5_CAPT_vect) {
  uint8_t cl = ICR5L;
  uint8_t ch = ICR5H;
  uint8_t ccl = TCNT5L;
  uint8_t cch = TCNT5H;
  TCCR5B ^= _BV(ICES1); // alternate direction of edge of next capture  
  // did we capture a rise or fall?
  uint8_t gotfall =   TCCR5B & _BV(ICES1);
  finish_capture_isr(gotfall,cl,ch,cch);
}

#endif


#if defined(model_UNO_NANO) || defined(model_MEGA)
ISR(TIMER1_OVF_vect) {
  ovfcount++; // each tick is worth 0.5us, each ovf worth 32.768ms
  if (edgeEventHead != edgeEventTail) {
    // if the latest entry is a timer tick that has not yet been seen,
    // overwrite it.  "Not been seen" means the circular buffer must
    // contain at least 2 entries: an earlier one that might be getting looked at,
    // plus a later one we might overwrite
    if (((edgeEventHead - edgeEventTail) & (edgeEventCount-1)) >= 2) {
      if (edgeEvents[edgeEventHead].portaddr==0) {
        edgeEventHead = (edgeEventHead-1) & (edgeEventCount-1);         
      }
    }    
  }
  ISRX(TIFR1, 0, TCNT1H, TCNT1L);
}
#endif


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



chipguy_softSerialRx::chipguy_softSerialRx(byte _pin, uint32_t _baud) {
	init(_pin, 'S');
	
	// Calculate baud multiplier factor.
	// This factor should take the number of microseconds of 1 bit time at our selected baud,
	// and scale it so that 1 bit time = 0x1000 (4096).
	// Example: 9600 bits/sec has a bit time of 104.17us, 
	baudmulfac = _baud / 240;
	
}


void chipguy_softSerialRx::_handle_edge(char edgeKind, uint32_t timediff32, uint16_t timediff) {



	if (inword==0 && edgeKind=='F') {
		inword=1;
		bitsreceived=0;
		// we will assume all bits are 1 until we receive them as zero, because
		// the last edge could come before the last bit on the timeline, all of
		// which will be 1's in that case.
		charbuf = 0xff;
		
	} else {
		timediff32 = timediff32 * baudmulfac;		
				// Anything from 0   to 1.5 bit times is 1 bit.
				// Anything from 1.5 to 2.5 bit times is 2 bits.
				// Pattern here: we'll add a half for rounding, truncate fraction,
				// and then move 0 to be 1.		
		timediff32 += 2048;  // half a bit time (scaled)
		timediff32 /= 4096;	// scale to whole bits
		byte bittimes = timediff32;	// clamp to [1,10]
	
		if (inword && edgeKind != 'T') {
			if (bittimes==0) bittimes=1;
			if (bittimes>=10) inword=0;
			else while (bittimes-- && bitsreceived < 9) {
				// if we got a rise, then we just finished getting some zeroes.
				// we already assume everything's a 1 until informed otherwise, which happens here.
				if (bitsreceived && edgeKind=='R') charbuf &= ~(1 << (bitsreceived-1));              
				bitsreceived++;        
			}
		} else if (inword && edgeKind=='T' && bittimes > 11) {
			// if we got a timeout condition and last edge was in the middle of a word, then we accept the assumption
			// that the rest of the bits are 1's, and assume all the bits to be received.
			bitsreceived=9;
		}  
	}
					
	if (bitsreceived >= 9) {
		byte newHead = (receive_head+1) % sizeof(receiveBuffer);
		if (newHead != receive_tail) {
			receiveBuffer[receive_head]=charbuf;
			receive_head=newHead;		
		}

		// if we had a rise, we're out of word (stop bit).  if fall, we're starting a new word.
		inword = edgeKind=='F' ? 1 : 0;
		bitsreceived=0;
		charbuf=0xff;
	}			
}

int chipguy_softSerialRx::read(void) {
	if (receive_head==receive_tail) return -1;
	byte newtail = receive_tail+1;
	if (newtail >= sizeof(receiveBuffer)) newtail = 0;		
	byte rv = receiveBuffer[receive_tail];
	receive_tail = newtail;
 	return rv;
}

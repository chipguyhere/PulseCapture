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



chipguy_WiegandRx::chipguy_WiegandRx(byte pinD0, byte pinD1) {
	helper.init(pinD0, '0');
	((chipguy_wiegand_helper*)(&helper))->d1instance=this;
	init(pinD1, 'W');
}

int chipguy_WiegandRx::begin(void) {
	// ensure begin gets called on both
	int rv = PulseCapture::begin();
	int rv2 = helper.begin();
	if (rv==rv2) return rv;
	if (rv==-1 || rv2==-1) return -1;
	return (rv < rv2) ? rv : rv2;
}


chipguy_wiegand_helper::chipguy_wiegand_helper() {
	
}



void chipguy_wiegand_helper::_handle_edge(char edgeKind, uint32_t timediff32, uint16_t timediff) {

	// send it to the d1 instance for handling, adding a bit flag to indicate
	// that it has been sent over.
	d1instance->lastTimestamp = lastTimestamp;
	d1instance->_handle_edge(edgeKind + 0x80, timediff32, timediff);
}

void chipguy_WiegandRx::_handle_edge(char edgeKind, uint32_t timediff32, uint16_t timediff) {

	helper.lastTimestamp = lastTimestamp;

	// if we have a 0, then we are looking at the 0-pin of a two-pin Wiegand setup.
	bool is0=false, is1=true;
	if (edgeKind < 0) {
		edgeKind -= 0x80;
		is0=true,is1=false;
	}

	// On Wiegand, the signal is idle high.  We get bits when pin0 or pin1 falls.
	// As long as its fall isn't "late" (compared to earlier bits), it's good anytime.
	// "A little bit late" -- we ignore the bit.  (between 5 and 65ms)
	// "A lot late" -- consider it a brand new message (65ms+)

	if (edgeKind=='F') {            
		if (inword==0 || timediff >= 65535) {
			inword=1;
			bitsreceived=1;
			capbuf=is1 ? 1 : 0;              
		} else if (timediff < 5000) {
			if (bitsreceived < 33) {
				// On a 34-bit Wiegand message we only want the middle 32 bits.
				// We will dump the last (34th) bit by not saving it.
				// The first bit will get dumped by capbuf only having room for 32 bits.
				capbuf <<= 1;             
				if (is1) capbuf |= 1;
			}
			bitsreceived++;
		}
	} else if (edgeKind=='T' && timediff > 25000) {            
		if (bitsreceived==4 || bitsreceived==26 || bitsreceived==34) {
			capturedMessage = capbuf;
			capturedBitCount=4;              
			if (bitsreceived==26) {
				capturedMessage = (capbuf >> 1) & 0xFFFFFF;
				capturedBitCount=24;
			} else if (bitsreceived==34) {
				capturedBitCount=32;                
			} 
		}
		bitsreceived=0;
		inword=0;              
	}

}

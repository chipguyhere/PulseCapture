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


chipguy_irReceiver::chipguy_irReceiver(byte _pin) {
	init(_pin, 'I');
}


void chipguy_irReceiver::_handle_edge(char edgeKind, uint32_t rcvtime, uint32_t timediff32, uint16_t timediff) {

/*	
	Serial.print(edgeKind);
	Serial.print(" r=");
	Serial.print(rcvtime);
	Serial.print(" d=");
	Serial.print(timediff32);
	Serial.print(" i=");
	Serial.print(inword);	
	*/

	if (edgeKind=='F') {            
		if (inword==0 || timediff >= 30000U) {    // first edge of a message?
			inword=1;
			bitsreceived=0;
		} else if (inword==1) { // second edge of a message?  (got the start bit, but we're before the first data bit)
			if (timediff > 2000 && timediff < 2500) {
				// todo: FIX this, put last rcv time in its own variable so not interfering with
				// use of capbuf to hold actual IR capture.

				if (lastMessageTimestamp) {				
					if (rcvtime - lastMessageTimestamp < 65536) {
						// got the signal that says button is being held down.
						// While inword==0, capbuf is borrowed to be the timestamp of the last good message.
						// for a repeat to be valid, we needed to receive a good message in the last 125ms,
						// and the non-ISR code needs to have picked up the original message to know what to repeat.
						if (capturedBitCount==0) {
								capturedBitCount=1;
								capturedMessage=1;
						}
						if (edgeKind=='T') {
				
							inword=0;
						}
						lastMessageTimestamp=rcvtime;
					} else {
						lastMessageTimestamp=0;
					}
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
				lastMessageTimestamp = rcvtime;    
				capbuf=0;          
				capturedBitCount=32;
				inword=0;           
					 
			}
		}
	} else if (edgeKind=='R') { // gotrise          
		// normal rises only come with a time diff of:
		// ~9000 for a sync bit
		// ~4500 for a sync bit on some other remote I saw
		// ~600 for a normal pulse
		// rises don't start messages because IR receiver is idle high.
		// rises can be shorter than expected in case of low light/contrast.            
		if (timediff > 11000) inword=0;
		if (inword==2 && (timediff < 300 || timediff > 750)) inword=0;
	} else if (edgeKind=='T' && timediff > 10000) {
		inword=0;
	
	}
	
//	Serial.print(' ');
//	Serial.println(inword);
}


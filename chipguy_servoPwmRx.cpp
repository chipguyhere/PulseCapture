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



chipguy_servoPwmRx::chipguy_servoPwmRx(byte _pin) {
	init(_pin, 'P');
	clockrate=20;
	
}

void chipguy_servoPwmRx::_handle_edge(char edgeKind, uint32_t timediff32, uint16_t timediff) {



	// CAPTURE FOR SERVO PWM
	// it's simple: on falls, if it looks like a valid PWM pulse, the message is its length.
	// Valid pulses are nominally 1000-2000us, but overshoots are allowed.
	if (edgeKind=='F' && timediff32 > 500000 && timediff32 < 2500000) {
		capturedMessage = timediff32;
	}	

}

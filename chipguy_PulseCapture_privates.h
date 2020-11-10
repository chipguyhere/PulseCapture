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


extern bool pulsecapture_began;
extern volatile PulseCapture *first_pulsecapture_instance;
extern volatile uint16_t ovfcount;
extern bool main_timer_is_micros;


void cgh_pulsecapture_add_tick_to_queue(uint32_t timestamp);
void cgh_pulsecapture_add_event_to_queue(uint8_t portRead, char portid, uint32_t timestamp);



/*
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

#include <AP_HAL/AP_HAL.h>
#include "AP_RangeFinder_NRA24.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;


/* 
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_NRA24::AP_RangeFinder_NRA24(RangeFinder::RangeFinder_State &_state,
                                                               AP_RangeFinder_Params &_params,
                                                               uint8_t serial_instance) :
    AP_RangeFinder_Backend(_state, _params)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance));
    }
}

/* 
   detect if a NRA24 Lightware rangefinder is connected. We'll detect by
   trying to take a reading on Serial. If we get a result the sensor is
   there.
*/
bool AP_RangeFinder_NRA24::detect(uint8_t serial_instance)
{
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Rangefinder, serial_instance) != nullptr;
}

// read - return last value measured by sensor
bool AP_RangeFinder_NRA24::get_reading(uint16_t &reading_cm)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the NRA24
    float sum = 0;
    uint16_t count = 0;
	uint16_t index = 0;
	bool can_read = false;
	int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t c = uart->read();
		//The start sequence
		if(c == 0xAA && can_read== false){
			linebuf_len = 0;
			can_read = true;
		}		
		if(can_read == true){
			linebuf[linebuf_len] = c;
			linebuf_len++;
			if(linebuf_len == 14){
				if (linebuf[2] == 0x0C && linebuf[3] == 0x07)
				{
					uint16_t Range= (linebuf[6]*0x100 + linebuf[7]);
					sum += Range;
					index++;
				}				
				can_read = false;
				linebuf_len = 0;
				count++;				
			}
		}		
	}
    if (count == 0) {
        return false;
    }
	if (index ==0){
		reading_cm = sum;
	}else{
		reading_cm = sum/index;
	}
    return true;
}

/* 
   update the state of the sensor
*/
void AP_RangeFinder_NRA24::update(void)
{
    if (get_reading(state.distance_cm)) {
        // update range_valid state based on distance measured
        state.last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - state.last_reading_ms > 200) {
        set_status(RangeFinder::RangeFinder_NoData);
    }
}

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
#include "uSharp_Backend.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <ctype.h>

extern const AP_HAL::HAL& hal;


uSharp_Backend::uSharp_Backend(uSharp &_usharp, uSharp::uSharp_State &_state,
									AP_SerialManager &serial_manager):
state(_state),
usharp(_usharp)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0));
    }
}


bool uSharp_Backend::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_Aerotenna_uSharp, 0) != nullptr;
}

// read - return last value measured by sensor
bool uSharp_Backend::get_reading(uSharp::uSharp_State &state_ref)
{
    if (uart == nullptr) {
        return false;
    }

    // read any available lines from the uLanding
    float sum[4] = {0,};
    uint16_t count[4] = {0,};
    uint8_t  index = 0;

    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        uint8_t c = uart->read();
        // ok, we have located start byte
        if ( c == 72 && index ==0 ) {
            linebuf_len = 0;
            index       = 1;
        }
        // now it is ready to decode index information
        if ( index == 1 ){
        	linebuf[linebuf_len] = c;
        	linebuf_len ++;
        	if ( linebuf_len == 4 ){
        		index = 0;
        		switch(linebuf[1]){
        		case 252:
        			sum[3] += ( linebuf[3]&0x7F ) *128 + ( linebuf[2]&0x7F );
        			count[3]++;
        			break;
        		case 253:
        			sum[0] += ( linebuf[3]&0x7F ) *128 + ( linebuf[2]&0x7F );
        			count[0]++;
        			break;
        		case 254:
        			sum[1] += ( linebuf[3]&0x7F ) *128 + ( linebuf[2]&0x7F );
        			count[1]++;
        			break;
        		default:
        			sum[2] += ( linebuf[3]&0x7F ) *128 + ( linebuf[2]&0x7F );
        			count[2]++;
        		}

        		linebuf_len = 0;
        	}
        }

    }

    if ( count[0] == 0 && count[1] == 0 && count[2] == 0 && count[3] == 0 ) {
        return false;
    }
    if ( count[0] != 0 ){
    	state_ref.distance[0] = 2.5 * sum[0] / count[0];
    }
    if ( count[1] != 0) {
    	state_ref.distance[1] = 2.5 * sum[1] / count[1];
    }
    if ( count[2] != 0) {
    	state_ref.distance[2] = 2.5 * sum[2] / count[2];
    }
    if ( count[3] != 0) {
    	state_ref.distance[3] = 2.5 * sum[3] / count[3];
    }


    return true;
}

/*
   update the state of the sensor
*/
void uSharp_Backend::update(void)
{
	get_reading(state);
}

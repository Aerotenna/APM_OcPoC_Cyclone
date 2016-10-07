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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>


class uSharp_Backend;

class uSharp{

public:
	friend class uSharp_Backend;

	uSharp(AP_SerialManager &_serial_manager);

    // detect and initialise any available rangefinders
    void init(void);

    // Should be called at around 10Hz from main loop
    void update(void);

    void detect_instance();

    uint16_t distance_cm() const {
        return state.distance[0];
    }
    uint16_t distance_cm(uint8_t instance) const {
        return state.distance[instance];
    }

    struct uSharp_State{
    	uint16_t distance[4];
    };

private:

    uSharp_Backend   *driver;

    AP_SerialManager &serial_manager;

    uint16_t         distance;

    uSharp_State     state;


};

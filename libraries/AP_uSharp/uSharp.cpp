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

#include "uSharp.h"
#include "uSharp_Backend.h"

extern const AP_HAL::HAL &hal;


uSharp::uSharp(AP_SerialManager &_serial_manager):
serial_manager(_serial_manager)
{
	driver = NULL;
}


void uSharp::init(void){
	detect_instance();

}

void uSharp::detect_instance(){
	if ( uSharp_Backend::detect(serial_manager)){
		driver = new uSharp_Backend(*this, state, serial_manager);
	}
}

void uSharp::update(){
	if ( driver != NULL ){
		driver->update();
	}
}

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "uSharp.h"

class uSharp_Backend
{

public:
    // constructor
	uSharp_Backend(uSharp &usharp, uSharp::uSharp_State &state,
						AP_SerialManager &serial_manager);

    // static detection function
    static bool detect(AP_SerialManager &serial_manager);

    // update state
    void update(void);

private:
    // get a reading
    bool get_reading(uSharp::uSharp_State &state_ref);

    AP_HAL::UARTDriver *uart = nullptr;
    uint32_t last_reading_ms = 0;
    uint8_t linebuf[10];
    uint8_t linebuf_len = 0;

    uSharp                 &usharp;
    uSharp::uSharp_State   &state;
};



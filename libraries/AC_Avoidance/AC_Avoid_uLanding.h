// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

/// @file    AC_Avoid_uLanding.h
/// @brief   ArduCopter obstacle avoidance with uLanding

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
//#include <AP_AHRS/AP_AHRS.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

#define ULANDING_ENABLE_DEFAULT 0
#define ULANDING_AVOID_DIST_DEFAULT      250.0f // cm
#define ULANDING_AVOID_DIST_BUFF_DEFAULT  50.0f  // cm
#define ULANDING_AVOID_DIST_VALID_DEFAULT 75.0f  // cm
#define ULANDING_PITCH_LIMIT 2000.0f  // centi-degrees
#define ULAND_STB_kP        15.0f
#define ULAND_STB_kI        0.1f
#define ULAND_STB_kD        0.0f
#define ULAND_STB_IMAX      200.0f    // centi-degrees
#define ULAND_STB_FILT_HZ   20.0f     // Hz

/*
 * This class commands the vehicle to avoid an obstacle
 * detected by a forward facing uLanding.
 */
class AC_Avoid_uLanding {
public:

    /// Constructor
    AC_Avoid_uLanding(const AP_Motors& motors, const RangeFinder& range, float dt);

    // monitor - monitor whether or not to avoid an obstacle
    bool monitor(void);

    // stabilize_avoid - returns new pitch command in centi-degrees to avoid obstacle
    void stabilize_avoid(float &pitch_cmd);
/*
    // loiter_avoid - returns new velocity commands in cm/s to avoid obstacle
    void loiter_avoid(void); 
*/
    static const struct AP_Param::GroupInfo var_info[];

private:

    // obstacle_detect - read uLanding and determine if obstacle is present and needs to be avoided
    bool obstacle_detect(float dist);

    // external references
    const AP_Motors&    _motors;
    const RangeFinder&  _range;

    // PID controllers
    AC_PID      _pid_stab_avoid;

    // parameters
    AP_Int8     _uLanding_avoid_enable;
    AP_Float    _uLanding_avoid_dist;
    AP_Float    _uLanding_avoid_dist_buffer;
    AP_Float    _uLanding_avoid_dist_valid;
    AP_Float    _uLanding_avoid_pitch_lim;

    // internal variables
    bool    _avoid;
    bool    _avoid_prev;
    float   _dt;
};

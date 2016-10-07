// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

/// @file    AC_Avoid_uSharp.h
/// @brief   ArduCopter obstacle avoidance with uSharp

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AC_PID/AC_PID.h>
#include <AP_uSharp/uSharp.h>

#define USHARP_ENABLE_DEFAULT 0
#define NUM_USHARP_PANELS 4
#define PANEL_AZ_1 (0.0f * M_PI / 180.0f)
#define PANEL_AZ_2 (90.0f * M_PI / 180.0f)
#define PANEL_AZ_3 (180.0f * M_PI / 180.0f)
#define PANEL_AZ_4 (270.0f * M_PI / 180.0f)
#define USHARP_AVOID_DIST_DEFAULT      250.0f // cm
#define USHARP_AVOID_DIST_BUFF_DEFAULT  50.0f  // cm
#define USHARP_AVOID_DIST_VALID_DEFAULT 75.0f  // cm
#define USHARP_LEAN_ANGLE_LIMIT 2000.0f  // centi-degrees
#define USHARP_STB_kP        15.0f
#define USHARP_STB_kI        0.1f
#define USHARP_STB_kD        0.0f
#define USHARP_STB_IMAX      200.0f    // centi-degrees
#define USHARP_STB_FILT_HZ   20.0f     // Hz

/*
 * This class commands the vehicle to avoid an obstacle
 * detected by a forward facing uSharp.
 */
class AC_Avoid_uSharp {
public:

    /// Constructor
    AC_Avoid_uSharp(const AP_Motors& motors, const uSharp& usharp, const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control, float dt);

    // monitor - monitor whether or not to avoid an obstacle
    bool monitor(void);

    // stabilize_avoid - returns new pitch command in centi-degrees to avoid obstacle
    //                 - Units: pitch_cmd, roll_cmd, and angle_max - centi-degrees
    void stabilize_avoid(float &pitch_cmd, float &roll_cmd, float angle_max);

    // loiter_avoid - returns new velocity commands in cm/s to avoid obstacle
    void loiter_avoid(float pitch_in, float roll_in, float &pitch_out, float &roll_out, float angle_max); 

    static const struct AP_Param::GroupInfo var_info[];

private:

    // update_loiter_target - move the target position in loiter mode to maintain 
    //                        body y-axis position/velocity command, but align 
    //                        body x-axis target position with current position
    void update_loiter_target(void);

    // calc_pitch_roll_err - calculate sum of pitch/roll axis errors for uSharp panels
    //                - units:  distance - cm
    //                          panel_azimuth - radians
    //                          pitch_err/roll_err - centi-degrees
    void calc_pitch_roll_err(void);

    // moved_past_buffer - check to see if previously detected obstacle is now
    //                     outside of buffer distance; return true if all obstacles
    //                     are now past the buffer distance
    bool moved_past_buffer(void);

    // adjust_pilot_cmd - adjust avoidance pitch/roll commands with component of pilot command
    //                    perpendicular to obstacle(s) we are avoiding 
    void add_pilot_cmd(float pilot_pitch, float pilot_roll, float &avoid_pitch, float &avoid_roll);

    // reset_integrators - resets pitch or roll integrators if current _avoid 
    //                     state is true, but previous _avoid state was false
    void reset_integrators(void);

    // read_usharp - update distance readings from uSharp panels
    void read_usharp(void);

    // obstacle_detect - read uSharp and determine if obstacle is present and needs to be avoided
    bool obstacle_detect(uint16_t dist);

    // update_buffer - update buffer distance based on user parameters
    void update_buffer(float dist, float buffer) { _buffer = dist + buffer;}

    // external references
    const AP_Motors&        _motors;
    const uSharp&           _usharp;
    const AP_InertialNav&   _inav;
    const AP_AHRS&          _ahrs;
    AC_PosControl&          _pos_control;

    // PID controllers
    AC_PID      _pid_avoid_pitch;
    AC_PID      _pid_avoid_roll;

    // parameters
    AP_Int8     _uSharp_avoid_enable;
    AP_Int8     _uSharp_looking_fwd;
    AP_Float    _uSharp_avoid_dist;
    AP_Float    _uSharp_avoid_dist_buffer;
    AP_Float    _uSharp_avoid_dist_valid;
    AP_Float    _uSharp_avoid_pitch_lim;

    // internal variables
    bool      _avoid[NUM_USHARP_PANELS];
    bool      _avoid_prev[NUM_USHARP_PANELS];
    bool      _run_avoid[NUM_USHARP_PANELS] = { false };
    float     _buffer;
    float     _dt;
    uint16_t  _distance_cm[NUM_USHARP_PANELS];
    float     _usharp_panel_azimuth[NUM_USHARP_PANELS] = {PANEL_AZ_1, PANEL_AZ_2, PANEL_AZ_3, PANEL_AZ_4};
};

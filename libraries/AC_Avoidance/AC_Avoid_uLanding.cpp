#include "AC_Avoid_uLanding.h"

const AP_Param::GroupInfo AC_Avoid_uLanding::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: uLanding Avoidance control enable/disable
    // @Description: Enabled/disable stopping based on uLanding feedback
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    AP_GROUPINFO("ENABLE",1, AC_Avoid_uLanding, _uLanding_avoid_enable, ULANDING_ENABLE_DEFAULT),

    // @Param: DIRECTION
    // @DisplayName: uLanding Avoidance direction
    // @Description: uLanding sensor looking forward or backward?
    // @Values: 0: Backward, 1: Forward
    // @User: Standard
    AP_GROUPINFO("DIRECTION",2, AC_Avoid_uLanding, _uLanding_looking_fwd, ULANDING_LOOKING_FWD),

    // @Param: DIST
    // @DisplayName: uLanding Avoidance standoff distance
    // @Description: uLanding distance to maintain from obstacle
    // @Units: cm
    // @Values: 0 1000.0
    // @User: Standard
    AP_GROUPINFO("DIST",3, AC_Avoid_uLanding, _uLanding_avoid_dist, ULANDING_AVOID_DIST_DEFAULT),

    // @Param: DIST_BUFF
    // @DisplayName: uLanding Avoidance buffer distance
    // @Description: uLanding distance required before exiting obstacle avoidance
    // @Units: cm
    // @Values: 0 1000.0
    // @User: Standard
    AP_GROUPINFO("DIST_BUFF",4, AC_Avoid_uLanding, _uLanding_avoid_dist_buffer, ULANDING_AVOID_DIST_BUFF_DEFAULT),

    // @Param: RNG_VALID
    // @DisplayName: uLanding Avoidance valid distance
    // @Description: minimum uLanding distance reading required before measurement is considered valid
    // @Units: cm
    // @Values: 31.0 100.0
    // @User: Standard
    AP_GROUPINFO("RNG_VALID",5, AC_Avoid_uLanding, _uLanding_avoid_dist_valid, ULANDING_AVOID_DIST_VALID_DEFAULT),

    // @Param: PIT_LIM
    // @DisplayName: uLanding Avoidance pitch limit
    // @Description: uLanding distance pitch limit
    // @Units: centi-degrees
    // @Values: 1000.0 4500.0 
    // @User: Standard
    AP_GROUPINFO("PIT_LIM",6, AC_Avoid_uLanding, _uLanding_avoid_pitch_lim, ULANDING_PITCH_LIMIT),

    // @Param: PIT_P
    // @DisplayName: Track Error controller P gain
    // @Description: Track Error controller P gain.  Converts track error into a velocity output (perpendicular to flight path)
    // @Range: 0.0 12.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: PIT_I
    // @DisplayName: Track Error controller I gain
    // @Description: Track Error controller I gain.  Corrects long-term difference in track error
    // @Range: 0.0 5.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: PIT_IMAX
    // @DisplayName: Track Error controller I gain maximum
    // @Description: Track Error controller I gain maximum.  Constrains the maximum rate output that the I gain will output
    // @Range: 0 100
    // @Increment: 0.01
    // @Units: cm/s
    // @User: Standard

    // @Param: PIT_D
    // @DisplayName: Track Error controller D gain
    // @Description: Track Error controller D gain.  Compensates for short-term change in track error
    // @Range: 0.0 5.0
    // @Increment: 0.001
    // @User: Standard

    // @Param: PIT_FILT
    // @DisplayName: Track Error conroller input frequency in Hz
    // @Description: Track Error conroller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    AP_SUBGROUPINFO(_pid_stab_avoid, "PIT_", 7, AC_Avoid_uLanding, AC_PID),


    AP_GROUPEND
};

/// Constructor
AC_Avoid_uLanding::AC_Avoid_uLanding(const AP_Motors& motors, const uSharp& usharp, const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control, float dt)
    : _motors(motors),
      _usharp(usharp),
      _inav(inav),
      _ahrs(ahrs),
      _pos_control(pos_control),
      _dt(dt),
      _pid_stab_avoid(ULAND_STB_kP, ULAND_STB_kI, ULAND_STB_kD, ULAND_STB_IMAX, ULAND_STB_FILT_HZ, dt)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialize avoidance flags
    _avoid = false;
    _avoid_prev = false;
}

// monitor - monitor whether or not to avoid an obstacle
bool AC_Avoid_uLanding::monitor(void)
{

    // If motors are unarmed, do nothing
    if (!_motors.armed() || !_motors.get_interlock() || !_uLanding_avoid_enable) {
        return false;
    }else{
        update_buffer(_uLanding_avoid_dist, _uLanding_avoid_dist_buffer);
        return obstacle_detect(_usharp.distance_cm());
    }
}

// stabilize_avoid - returns new pitch command in centi-degrees to avoid obstacle
void AC_Avoid_uLanding::stabilize_avoid(float &pitch_cmd)
{
    // Reset integrator if the previous avoidance state was false
    if (!_avoid_prev) {
        _pid_stab_avoid.reset_I();
    }

    // determine if pilot is commanding pitch to back away from obstacle
    //  - includes a dead-zone of 5 degrees
    bool pilot_cmd_avoidance;
    if (_uLanding_looking_fwd) {
    	pilot_cmd_avoidance = pitch_cmd > 250.0;
    }else{
        pilot_cmd_avoidance = pitch_cmd < -250.0;
    }

    if (pilot_cmd_avoidance || (_usharp.distance_cm() > _buffer)) {
        // allow pilot to maintain pitch command if actively avoiding obstacle
        pitch_cmd = pitch_cmd;
    }else{

        // calcualate distance error
        float err = _uLanding_avoid_dist - _usharp.distance_cm();

        if (!_uLanding_looking_fwd) {
            // if the uLanding sensor is looking backward, flip sign of the error to produce opposite pitch cmd
            err = -err;
        }

        // pass error to the PID controller for avoidance distance
        _pid_stab_avoid.set_input_filter_d(err);

        // compute pitch command from pid controller
        pitch_cmd = _pid_stab_avoid.get_pi();

        // limit pitch_cmd
        pitch_cmd = constrain_float(pitch_cmd, -_uLanding_avoid_pitch_lim, _uLanding_avoid_pitch_lim);
    }

    // set previous avoid state for next step through the monitor
    if (_usharp.distance_cm() > _buffer) {
            _avoid = false;
    }

    _avoid_prev = _avoid;
}

// loiter_avoid - currently a place holder

void AC_Avoid_uLanding::loiter_avoid(float pitch_in, float &pitch_out)
{
    // Reset integrator if the previous avoidance state was false
    if (!_avoid_prev) {
        // reset integrator if entering avoidance for the first time
        _pid_stab_avoid.reset_I();
    }

    // update target position for loiter mode, since we're actively
    // avoiding an obstacle
    update_loiter_target();

    // determine if pilot is commanding pitch to back away from obstacle
    //  - includes a dead-zone of 5 degrees
    bool pilot_cmd_avoidance;
    if (_uLanding_looking_fwd) {
    	pilot_cmd_avoidance = pitch_in > 250.0;
    }else{
        pilot_cmd_avoidance = pitch_in < -250.0;
    }

    if (pilot_cmd_avoidance || (_usharp.distance_cm() > _buffer)) {
        // allow pilot to maintain pitch command if actively avoiding obstacle
        pitch_out = pitch_in;
    }else{

        // calcualate distance error
        float err = _uLanding_avoid_dist - _usharp.distance_cm();

        if (!_uLanding_looking_fwd) {
            // if the uLanding sensor is looking backward, flip sign of the error to produce opposite pitch cmd
            err = -err;
        }

        // pass error to the PID controller for avoidance distance
        _pid_stab_avoid.set_input_filter_d(err);

        // compute pitch command from pid controller
        pitch_out = _pid_stab_avoid.get_pi();

        // limit pitch_cmd
        pitch_out = constrain_float(pitch_out, -_uLanding_avoid_pitch_lim, _uLanding_avoid_pitch_lim);
    }

    // set previous avoid state for next step through the monitor
    if (_usharp.distance_cm() > _buffer) {
            _avoid = false;
    }

    _avoid_prev = _avoid;
}

// update_loiter_target - move the target position in loiter mode to maintain body y-axis position/velocity command,
//                      but align body x-axis target position with current position
void AC_Avoid_uLanding::update_loiter_target(void)
{
    Vector3f curr_pos = _inav.get_position();
    Vector3f pos_targ = _pos_control.get_pos_target();
    float heading = wrap_180_cd(_ahrs.yaw_sensor);
    float distToDest;
    float courseToDest;
    float distToMoveTarget;
    Vector2f new_target;

    // calculate distance to destination
    distToDest = norm((pos_targ.y - curr_pos.y),(pos_targ.x - curr_pos.x));

    // calculate course to destination
    courseToDest = atan2f((pos_targ.y - curr_pos.y),(pos_targ.x - curr_pos.x));

    // calculate the distance to move the target from the current position
    distToMoveTarget = distToDest * sinf(courseToDest - heading);

    // only move the target position if it is behind the obstacle we're avoiding
    if ((_uLanding_looking_fwd && (abs(courseToDest - heading) < M_PI_2)) ||
       (!_uLanding_looking_fwd && (abs(courseToDest - heading) > M_PI_2))) {

        // calculate new target x/y position, 
        // offset from current position in the body y-axis direction
        new_target.x = curr_pos.x - distToMoveTarget * _ahrs.sin_yaw();//sinf(heading);
        new_target.y = curr_pos.y + distToMoveTarget * _ahrs.cos_yaw();//cosf(heading);

        // set new target position
        _pos_control.set_xy_target(new_target.x, new_target.y);
    }
}

// obstacle_detect - read uLanding and determine if obstacle is present and needs to be avoided
// 			dist is read in cm
bool AC_Avoid_uLanding::obstacle_detect(uint16_t dist)
{
    bool uLanding_valid = dist > _uLanding_avoid_dist_valid; // valid distance in cm

    if (uLanding_valid && (dist <= _uLanding_avoid_dist)) {
        _avoid = true;
    }else if (!uLanding_valid && _avoid_prev) {
        _avoid = false;
        _avoid_prev = false;
    }

    bool run_avoid = uLanding_valid && _avoid;

    return run_avoid;
}

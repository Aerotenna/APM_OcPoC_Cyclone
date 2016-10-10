#include "AC_Avoid_uSharp.h"

const AP_Param::GroupInfo AC_Avoid_uSharp::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: uSharp Avoidance control enable/disable
    // @Description: Enabled/disable stopping based on uSharp feedback
    // @Values: 0:Disable,1:Enable
    // @User: Standard
    AP_GROUPINFO("ENABLE",1, AC_Avoid_uSharp, _uSharp_avoid_enable, USHARP_ENABLE_DEFAULT),

    // @Param: DIST
    // @DisplayName: uSharp Avoidance standoff distance
    // @Description: uSharp distance to maintain from obstacle
    // @Units: cm
    // @Values: 0 1000.0
    // @User: Standard
    AP_GROUPINFO("DIST",2, AC_Avoid_uSharp, _uSharp_avoid_dist, USHARP_AVOID_DIST_DEFAULT),

    // @Param: DIST_BUFF
    // @DisplayName: uSharp Avoidance buffer distance
    // @Description: uSharp distance required before exiting obstacle avoidance
    // @Units: cm
    // @Values: 0 1000.0
    // @User: Standard
    AP_GROUPINFO("DIST_BUFF",3, AC_Avoid_uSharp, _uSharp_avoid_dist_buffer, USHARP_AVOID_DIST_BUFF_DEFAULT),

    // @Param: RNG_VALID
    // @DisplayName: uSharp Avoidance valid distance
    // @Description: minimum uSharp distance reading required before measurement is considered valid
    // @Units: cm
    // @Values: 31.0 100.0
    // @User: Standard
    AP_GROUPINFO("RNG_VALID",4, AC_Avoid_uSharp, _uSharp_avoid_dist_valid, USHARP_AVOID_DIST_VALID_DEFAULT),

    // @Param: ANGLE_LIM
    // @DisplayName: uSharp Avoidance pitch/roll cmd limit
    // @Description: uSharp Avoidance pitch/roll cmd limit
    // @Units: centi-degrees
    // @Values: 1000.0 4500.0 
    // @User: Standard
    AP_GROUPINFO("ANGLE_LIM",5, AC_Avoid_uSharp, _uSharp_avoid_angle_lim, USHARP_LEAN_ANGLE_LIMIT),

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
    AP_SUBGROUPINFO(_pid_avoid_pitch, "PIT_", 6, AC_Avoid_uSharp, AC_PID),

    // @Param: RLL_P
    // @DisplayName: Track Error controller P gain
    // @Description: Track Error controller P gain.  Converts track error into a velocity output (perpendicular to flight path)
    // @Range: 0.0 12.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RLL_I
    // @DisplayName: Track Error controller I gain
    // @Description: Track Error controller I gain.  Corrects long-term difference in track error
    // @Range: 0.0 5.0
    // @Increment: 0.01
    // @User: Standard

    // @Param: RLL_IMAX
    // @DisplayName: Track Error controller I gain maximum
    // @Description: Track Error controller I gain maximum.  Constrains the maximum rate output that the I gain will output
    // @Range: 0 100
    // @Increment: 0.01
    // @Units: cm/s
    // @User: Standard

    // @Param: RLL_D
    // @DisplayName: Track Error controller D gain
    // @Description: Track Error controller D gain.  Compensates for short-term change in track error
    // @Range: 0.0 5.0
    // @Increment: 0.001
    // @User: Standard

    // @Param: RLL_FILT
    // @DisplayName: Track Error conroller input frequency in Hz
    // @Description: Track Error conroller input frequency in Hz
    // @Range: 1 100
    // @Increment: 1
    // @Units: Hz
    AP_SUBGROUPINFO(_pid_avoid_roll, "RLL_", 7, AC_Avoid_uSharp, AC_PID),


    AP_GROUPEND
};

/// Constructor
AC_Avoid_uSharp::AC_Avoid_uSharp(const AP_Motors& motors, const uSharp& usharp, const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control, float dt)
    : _motors(motors),
      _usharp(usharp),
      _inav(inav),
      _ahrs(ahrs),
      _pos_control(pos_control),
      _dt(dt),
      _pid_avoid_pitch(USHARP_STB_kP, USHARP_STB_kI, USHARP_STB_kD, USHARP_STB_IMAX, USHARP_STB_FILT_HZ, dt),
      _pid_avoid_roll(USHARP_STB_kP, USHARP_STB_kI, USHARP_STB_kD, USHARP_STB_IMAX, USHARP_STB_FILT_HZ, dt)
{
    AP_Param::setup_object_defaults(this, var_info);

}

// monitor - monitor whether or not to avoid an obstacle
bool AC_Avoid_uSharp::monitor(void)
{

    // If motors are unarmed, do nothing
    if (!_motors.armed() || !_motors.get_interlock() || !_uSharp_avoid_enable) {
        return false;
    }else{
        update_buffer(_uSharp_avoid_dist, _uSharp_avoid_dist_buffer);
        read_usharp();
        return obstacle_detect();
    }
}

// stabilize_avoid - returns new pitch command in centi-degrees to avoid obstacle
//                 - Units: pitch_cmd, roll_cmd, and angle_max - centi-degrees
void AC_Avoid_uSharp::stabilize_avoid(float &pitch_cmd, float &roll_cmd, float angle_max)
{
    // check if integrators need to be reset
    reset_integrators();

    if ( moved_past_buffer() ) {
        // allow pilot to maintain pitch command if all measurements are past the buffer distance
        pitch_cmd = pitch_cmd;
        roll_cmd  = roll_cmd;

    }else{

        // get pitch/roll error calculation
        Vector2f lean_angle_err = calc_pitch_roll_err();


        // pass error to the PID controllers for avoidance distance
        _pid_avoid_pitch.set_input_filter_d(lean_angle_err.x);
        _pid_avoid_roll.set_input_filter_d(lean_angle_err.y);

        // compute avoidance pitch/roll commands from pid controller
        float avoid_pitch_cmd = constrain_float(_pid_avoid_pitch.get_pi(), -_uSharp_avoid_angle_lim, _uSharp_avoid_angle_lim);
        float avoid_roll_cmd  = constrain_float(_pid_avoid_roll.get_pi(), -_uSharp_avoid_angle_lim, _uSharp_avoid_angle_lim);

        // add allowable compenent(s) of pilot commands to the avoidance-
        // synthesized pitch/roll commands
        add_pilot_cmd(pitch_cmd, roll_cmd, avoid_pitch_cmd, avoid_roll_cmd);

        // do circular limit
        float total_in = norm(avoid_pitch_cmd, avoid_roll_cmd);
        if (total_in > angle_max) {
            float ratio = angle_max / total_in;
            avoid_roll_cmd  *= ratio;
            avoid_pitch_cmd *= ratio;
        }

        // do lateral tilt to euler roll conversion
        avoid_roll_cmd = (18000/M_PI) * atanf(cosf(avoid_pitch_cmd*(M_PI/18000))*tanf(avoid_roll_cmd*(M_PI/18000)));

        // set final pitch/roll commands
        pitch_cmd = avoid_pitch_cmd;
        roll_cmd  = avoid_roll_cmd;
    }

}

// loiter_avoid - convenience function to avoid messing with the input of wp_nav.get_pitch()/wp_nav.get_roll()

void AC_Avoid_uSharp::loiter_avoid(float pitch_in, float roll_in, float &pitch_out, float &roll_out, float angle_max)
{
    // pitch_out and roll_out are used as temporary variables and defined as 0.0. 
    // set them to the wp_nav pitch/roll commands, which we will regard as similar
    // to pilot commands
    pitch_out = pitch_in;
    roll_out  = roll_in;

    // use the stabilize_avoid function to combine loiter mode commands
    // with obstacle avoidance commands
    stabilize_avoid(pitch_out, roll_out, angle_max);
}

// update_loiter_target - move the target position in loiter mode to maintain body y-axis position/velocity command,
//                      but align body x-axis target position with current position
void AC_Avoid_uSharp::update_loiter_target(void)
{
    Vector3f curr_pos = _inav.get_position();
    Vector3f pos_targ = _pos_control.get_pos_target();
    float heading = wrap_PI( radians(_ahrs.yaw_sensor / 100.0f) );
    Vector2f new_target;

    // calculate distance to destination
    float distToDest = norm((pos_targ.y - curr_pos.y),(pos_targ.x - curr_pos.x));
    float tmp_dist   = distToDest;

    // calculate course to destination
    float courseToDest = atan2f((pos_targ.y - curr_pos.y),(pos_targ.x - curr_pos.x));

    // calculate azimuth of destination relative to quad's heading
    float dest_azimuth = wrap_2PI( courseToDest - heading );
    float tmp_azimuth = dest_azimuth;

    for (uint8_t i=0; i<NUM_USHARP_PANELS; i++) {

        // calculate difference of azimuth angle between current
        // tmp_cmd (initialized as pilot's cmd) and the uSharp's 
        // azimuth
        float angle_diff = tmp_azimuth - _usharp_panel_azimuth[i];

        // adjust angle_diff for wrap from 0 - 360 deg
        if (angle_diff > M_PI_2)
            angle_diff -= M_2PI;

        // adjust loiter's pos_target if current uSharp panel sees
        // an object AND the current tmp_azimuth is 
        // within 90 deg of the panel
        if ( (_avoid[i] || _avoid_prev[i]) &&
              abs(angle_diff) <= M_PI_2 ) {

            // adjust temporary pos_target change to only allow
            // pos_target changes perpendicular to the uSharp panel
            //  - ex: if forward looking panel sees object,
            //        only allow pos_target changes in the roll axis
            tmp_dist *= abs( sinf(angle_diff) );

            // adjust tmp_azimuth perpendicular to current uSharp panel
            tmp_azimuth = angle_diff >= 0 ? \
                          wrap_2PI( _usharp_panel_azimuth[i] + M_PI_2) : \
                          wrap_2PI( _usharp_panel_azimuth[i] - M_PI_2);
        }
    }


    // only move the target position if it is behind the obstacle we're avoiding
    if (abs(wrap_PI(tmp_azimuth - dest_azimuth)) > M_PI_2) {
        // if the final adjusted tmp_azimuth is rotated more than
        // 90 deg from pilot's command, reset loiter's pos_target to current position

        _pos_control.set_xy_target(curr_pos.x, curr_pos.y);
        return;
    }else{
        // add allowable component of loiter's pos_target as an
        // offset from the quad's current position
        new_target.x = curr_pos.x + tmp_dist * cosf( heading + wrap_PI(tmp_azimuth) );//_ahrs.sin_yaw();
        new_target.y = curr_pos.y + tmp_dist * sinf( heading + wrap_PI(tmp_azimuth) );//_ahrs.cos_yaw();

        // set new target position
        _pos_control.set_xy_target(new_target.x, new_target.y);
    }
}

// calc_pitch_roll_err - calculate sum of pitch/roll axis errors for uSharp panels
//                - units:  distance - cm
//                          panel_azimuth - radians
//                          pitch_err/roll_err - centi-degrees
Vector2f AC_Avoid_uSharp::calc_pitch_roll_err(void)
{
    float tmp_err;
    float pitch_err = 0.0f;
    float roll_err  = 0.0f;

    for (uint8_t i=0; i<NUM_USHARP_PANELS; i++) {

        // calculate error between avoidance distance and the uSharp distance measurement
        tmp_err = _distance_cm[i] - _uSharp_avoid_dist;

        // apply the error calculation to the pitch and roll axes based on
        // the azimuth of the uSharp panel
        pitch_err -= tmp_err * cosf(_usharp_panel_azimuth[i]);
        roll_err  += tmp_err * sinf(_usharp_panel_azimuth[i]);
    }

    return Vector2f(pitch_err, roll_err);
}

// moved_past_buffer - check to see if previously detected obstacle is now
//                     outside of buffer distance
//                   - assumes we are in avoidance mode for at least one panel, resets
//                     _avoid/_avoid_prev flags for each panel accordingly
//                   - return true if all obstacles are now past the buffer distance
bool AC_Avoid_uSharp::moved_past_buffer(void)
{
    int count = 0;
    bool beyond_buffer = false;

    for (uint8_t i=0; i<NUM_USHARP_PANELS; i++) {

        if (_avoid[i] || _avoid_prev[i]) {

            if (_distance_cm[i] > _buffer) {
                // if we were avoiding an obstacle in front of this uSharp
                // panel, but its reading is beyond the buffer distance,
                // reset the panel's _avoid flag.
                _avoid[i] = false;
            }else{
                // increment the count if we're still avoiding an obstacle
                // in the direction of this uSharp panel
                count++;
            }
        }
    // set this uSharp panel's _avoid_prev flag
    _avoid_prev[i] = _avoid[i];
    }

    if (count == 0)
        // count == 0 means all uSharp panels that saw an obstacle now read
        // a measurement beyond the buffer distance
        beyond_buffer = true;

    return beyond_buffer;
}

// adjust_pilot_cmd - adjust avoidance pitch/roll commands with component of pilot command
//                    perpendicular to obstacle(s) we are avoiding 
void AC_Avoid_uSharp::add_pilot_cmd(float pilot_pitch, float pilot_roll, float &avoid_pitch, float &avoid_roll)
{
    // use negative pilot_pitch_cmd to translate pitch command into azimuth
    // angle relative to drone body axis
    float pilot_cmd_azimuth = wrap_2PI( atan2f(pilot_roll, -pilot_pitch) );
    float pilot_cmd_lean_angle = norm(pilot_pitch, pilot_roll);
    float tmp_azimuth = pilot_cmd_azimuth;
    float tmp_lean = pilot_cmd_lean_angle;

    // cycle through uSharp panels
    for (uint8_t i=0; i<NUM_USHARP_PANELS; i++) {

        // calculate difference of azimuth angle between current
        // tmp_cmd (initialized as pilot's cmd) and the uSharp's 
        // azimuth
        float angle_diff = tmp_azimuth - _usharp_panel_azimuth[i];

        // adjust angle_diff for wrap from 0 - 360 deg
        if (angle_diff > M_PI_2)
            angle_diff -= M_2PI;

        // adjust pilot's command if current uSharp panel sees
        // an object AND the current tmp_cmd's azimuth is 
        // within 90 deg of the panel
        if ( (_avoid[i] || _avoid_prev[i]) &&
              abs(angle_diff) <= M_PI_2 ) {

            // adjust tmp_cmd's lean angle to only allow
            // lean angles perpendicular to the uSharp panel
            //  - ex: if forward looking panel sees object,
            //        only allow the roll component of the
            //        pilot's command.
            tmp_lean *= abs( sinf(angle_diff) );

            // adjust tmp_azimuth perpendicular to current uSharp panel
            tmp_azimuth = angle_diff >= 0 ? \
                          wrap_2PI( _usharp_panel_azimuth[i] + M_PI_2) : \
                          wrap_2PI( _usharp_panel_azimuth[i] - M_PI_2);

        }
    }

    if (abs(wrap_PI(tmp_azimuth - pilot_cmd_azimuth)) > M_PI_2) {
        // if the final adjusted tmp_azimuth is rotated more than
        // 90 deg from pilot's command, ignore pilot's command and
        // maintain pitch/roll commands from avoidance algorithm
        return;
    }else{
        // add allowable part of pilot's command to the
        // pitch/roll commands from avoidance algorithm 
        avoid_pitch -= (tmp_lean * cosf(tmp_azimuth));
        avoid_roll += (tmp_lean * sinf(tmp_azimuth));
    }
}

// reset_integrators - resets pitch or roll integrators if current _avoid 
//                     state is true, but previous _avoid state was false
void AC_Avoid_uSharp::reset_integrators(void)
{
    bool reset_pitch = false;
    bool reset_roll = false;

    for (uint8_t i=0; i<NUM_USHARP_PANELS; i++) {

        if (_avoid[i] && !_avoid_prev[i]) {

            if (abs( cosf(_usharp_panel_azimuth[i]) ) > 0)
                reset_pitch = true;

            if (abs( sinf(_usharp_panel_azimuth[i]) ) > 0)
                reset_roll = true;
        }
    }

    if (reset_pitch)
        _pid_avoid_pitch.reset_I();

    if (reset_roll)
        _pid_avoid_roll.reset_I();
}

// read_usharp - update distance readings from uSharp panels
void AC_Avoid_uSharp::read_usharp(void)
{
    for (uint8_t i=0; i<NUM_USHARP_PANELS; i++) {
        _distance_cm[i] = _usharp.distance_cm(i);
    }
}

// obstacle_detect - determine if obstacle is present and needs to be avoided,
// 			         uses _distance_cm updated from read_usharp function
bool AC_Avoid_uSharp::obstacle_detect(void)
{
    // loop through distance readings from uSharp panels to determine if we need
    // to run the avoidance algorithm
    for (uint8_t i=0; i<NUM_USHARP_PANELS; i++) {

        // determine if the distance reading is considered valid
        bool uSharp_valid = _distance_cm[i] > _uSharp_avoid_dist_valid;

        if (uSharp_valid && (_distance_cm[i] <= _uSharp_avoid_dist)) {
            // set _avoid flag if uSharp reading is within the avoidance distance
            _avoid[i] = true;
        }else if (!uSharp_valid && _avoid_prev[i]) {
            // if the distance reading is no longer valid (after having gone through
            // avoidance algorithm), reset both _avoid and _avoid_prev flags
            _avoid[i] = false;
            _avoid_prev[i] = false;
        }

        // only run avoidance algorithm if we have a valid distance measurement and
        // there is an obstacle within the avoidance distance
        _run_avoid[i] = uSharp_valid && _avoid;
    }

    int  sum = 0;

    // cycle through run_avoid flags for all uSharp panels
    for (uint8_t i=0; i<NUM_USHARP_PANELS; i++) {
        if (_run_avoid[i])
            sum++;
    }

    // return true (run avoidance algorithm) if any uSharp reading
    // sets run_avoid to true
    if (sum > 0) {
        return true;
    }else{
        return false;
    }
}

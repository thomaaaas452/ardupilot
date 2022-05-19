#include "Copter.h"

#if MODE_SEMIAUTO_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeSemiAuto::init(bool ignore_checks)
{
    if (!copter.failsafe.radio) {
        float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(target_roll, target_pitch, loiter_nav->get_angle_max_cd(), attitude_control->get_althold_lean_angle_max_cd());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }
    loiter_nav->init_target();

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    return true;
}


// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeSemiAuto::run()
{
    float range_rf_cm = copter.rangefinder.distance_cm_orient(ROTATION_YAW_45);
    float range_rd_cm = copter.rangefinder.distance_cm_orient(ROTATION_YAW_135);
    float range_front_cm = copter.k210.cz/10;

    float range_r_min_cm = range_rf_cm>range_rd_cm ? range_rd_cm : range_rf_cm;

    float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        
        float angle_limit = attitude_control->get_althold_lean_angle_max_cd();
            // throttle failsafe check
        if (copter.failsafe.radio || !copter.ap.rc_receiver_present) {
            target_roll = 0;
            target_pitch = 0;
            return;
        }
        // fetch roll and pitch inputs

        if((range_rf_cm>120)&&(range_rd_cm>120))
        {
            target_roll = 350;
        }
        else if((range_rf_cm<80)&&(range_rd_cm<80))
        {
            target_roll = -350;
        }
        else{
            target_roll = 0;
        }

        if(target_roll==0)
        {
            if(range_rf_cm - range_rd_cm > 15.0)
            {
                target_yaw_rate = 0.1;
            }
            else if(range_rf_cm - range_rd_cm < -15.0)
            {
                target_yaw_rate = -0.1;
            }
            else {
                target_yaw_rate = 0;
            }
        }

        if(range_front_cm < 70){
            target_pitch = 600;
        }
        else if(range_front_cm>120){
            target_pitch = -400;
        }


        // target_roll = channel_roll->get_control_in();
        // target_pitch = channel_pitch->get_control_in();

        // limit max lean angle
        angle_limit = constrain_float(angle_limit, 1000.0f, loiter_nav->get_angle_max_cd());

        // scale roll and pitch inputs to ANGLE_MAX parameter range
        float scaler = loiter_nav->get_angle_max_cd()/(float)ROLL_PITCH_YAW_INPUT_MAX;
        target_roll *= scaler;
        target_pitch *= scaler;

        // do circular limit
        float total_in = norm(target_pitch, target_roll);
        if (total_in > angle_limit) {
            float ratio = angle_limit / total_in;
            target_roll *= ratio;
            target_pitch *= ratio;
        }

        // do lateral tilt to euler roll conversion
        target_roll = (18000/M_PI) * atanf(cosf(target_pitch*(M_PI/18000))*tanf(target_roll*(M_PI/18000)));

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

        // get pilot's desired yaw rate
        // target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // get pilot desired climb rate
        // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // clear out pilot desired acceleration in case radio failsafe event occurs and we do not switch to RTL for some reason
        loiter_nav->clear_pilot_desired_acceleration();
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // call attitude controller
        attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);


    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

#endif

#include "Copter.h"

#if MODE_SEMIAUTO_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeSemiAuto::init(bool ignore_checks)
{
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        float roll_out  = 0;
        float pitch_out = 0;
        float angle_max = copter.aparm.angle_max;
        float angle_limit = attitude_control->get_althold_lean_angle_max_cd();

        // limit max lean angle
        angle_limit = constrain_float(angle_limit, 1000.0f, angle_max);

        // scale roll and pitch inputs to ANGLE_MAX parameter range
        float scaler = angle_max/(float)ROLL_PITCH_YAW_INPUT_MAX;
        roll_out *= scaler;
        pitch_out *= scaler;

        // do circular limit
        float total_in = norm(pitch_out, roll_out);
        if (total_in > angle_limit) {
            float ratio = angle_limit / total_in;
            roll_out *= ratio;
            pitch_out *= ratio;
            }

        // do lateral tilt to euler roll conversion
        roll_out = (18000/M_PI) * atanf(cosf(pitch_out*(M_PI/18000))*tanf(roll_out*(M_PI/18000)));


        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(roll_out, pitch_out);
    
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

void ModeSemiAuto::run()
{
    int16_t target_x = copter.k210.cx;
    int16_t target_y = copter.k210.cy;
    static uint32_t change_time_ms = 0;

    float range_rf_cm = copter.rangefinder.distance_cm_orient(ROTATION_YAW_45);
    float range_rd_cm = copter.rangefinder.distance_cm_orient(ROTATION_YAW_135);
    float range_front_cm = copter.k210.cz/10;
    float range_ave = (range_rd_cm + range_rf_cm)*0.5;

    // if((target_x != 159)&&(target_y != 120))
    // {
        // copter.set_mode(Mode::Number::ATLO, ModeReason::MISSION_END);
    // }

    if((target_x == 159)&&(target_y == 120))
    {
        change_time_ms = millis();
    } else {
        if(millis()-change_time_ms>500)
        {
            copter.set_mode(Mode::Number::ATLO, ModeReason::MISSION_END);
        }
    }

    // gcs().send_text(MAV_SEVERITY_CRITICAL,
    //             "SEMIAUTO F:%.1f %d %d",
    //             range_front_cm,
    //             target_x,
    //             target_y);

    float range_r_min_cm = range_rf_cm>range_rd_cm ? range_rd_cm : range_rf_cm;

    float target_roll = 0.0f;
    float target_pitch = 0.0f;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;

    if(abs(range_rf_cm - range_rd_cm)>=250.0f)
    {
        range_ave = range_r_min_cm;
    }

    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);


        update_simple_mode();

        if((range_ave > g2.half_rdist_cm))
        {
            target_roll = g2.half_roll*(range_ave-g2.half_rdist_cm)*0.025 + 50;
            if(abs(target_roll)-abs(g2.half_roll)>=0.01f){
                target_roll = g2.half_roll;
            }
        } else if(range_ave <= g2.half_rdist_cm*0.9){
            target_roll = -g2.half_roll;
        } else {
            target_roll = 0;
        }
        if(range_r_min_cm < 80.0f)
        {
            target_roll = -1*g2.half_roll;
        }

//前后测距仪离1m时墙的距离，前面1642.84，后面1971.94
        if(abs(target_pitch) - abs(g2.half_pitch)*0.53<0.01f)
        // if(abs(target_roll) - abs(g2.half_roll)*0.72 < 0.01f)
        {
            if(range_rd_cm - range_rf_cm > g2.half_yawd)
            {
                // target_yaw_rate = g2.half_yaw*(range_rd_cm - range_rf_cm)/60;
                // if(abs(target_yaw_rate)-abs(g2.half_yaw)>=0.01f){
                //     target_yaw_rate = g2.half_yaw;
                // }
                // else if(abs(target_yaw_rate)-0.5*abs(g2.half_yaw)<=0.01f){
                //     target_yaw_rate = 0.5*g2.half_yaw;
                // } 
                target_yaw_rate = -100;         
            } else if(range_rd_cm - range_rf_cm <= -1*g2.half_yawd)
            {
                // target_yaw_rate = g2.half_yaw*(range_rd_cm - range_rf_cm)/60;
                // if(abs(target_yaw_rate)-abs(g2.half_yaw)>=0.01f){
                //     target_yaw_rate = -g2.half_yaw;
                // }
                // else if(abs(target_yaw_rate)-0.5*abs(g2.half_yaw)<=0.01f){
                //     target_yaw_rate = -0.5*g2.half_yaw;
                // }
                target_yaw_rate = 100;
            } else {
                    target_yaw_rate = 0;
            }
        } else{
            target_yaw_rate = 0;
        }
        

        // if(abs(target_roll) - 250 < 0.01f){
        if(abs(target_roll) - abs(g2.half_roll)*0.72 < 0.01f){    
            if(range_front_cm > g2.half_fdist_cm){
                // target_pitch = g2.half_pitch*(range_front_cm - g2.half_fdist_cm) * 0.02;
                target_pitch = g2.half_pitch;
                if(abs(target_pitch)-abs(g2.half_pitch)>=0.01f){
                    target_pitch = g2.half_pitch;
                }
            } else if(range_front_cm <= 0.85*g2.half_fdist_cm){
                target_pitch = -g2.half_pitch;
            } else{
                target_pitch = 0;
            }
        } 
        // else {
        //     target_pitch = 0;
        // }
        else {
            if(range_front_cm > g2.half_fdist_cm){
                // target_pitch = g2.half_pitch*(range_front_cm - g2.half_fdist_cm) * 0.02 * 0.75;
                target_pitch = g2.half_pitch * 0.75;
                if(abs(target_pitch)-abs(g2.half_pitch)*0.75>=0.01f){
                    target_pitch = g2.half_pitch*0.75;
                }
            } else if(range_front_cm <= 0.85*g2.half_fdist_cm){
                target_pitch = -g2.half_pitch;
            } else{
                target_pitch = 0;
            }
        }

        // float t_roll = target_roll;
        // float t_pitch = target_pitch;
        // float t_yaw = target_yaw_rate;


        // target_roll = channel_roll->get_control_in();
        // target_pitch = channel_pitch->get_control_in();

        float angle_limit = attitude_control->get_althold_lean_angle_max_cd();

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

        target_yaw_rate = get_pilot_desired_yaw_rate(target_yaw_rate/1000.0);

        gcs().send_text(MAV_SEVERITY_CRITICAL,
            "SEAU %.2f",
            range_front_cm
            );

        pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(target_roll, target_pitch);

        // get pilot's desired yaw rate
        // 

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(int(484));

        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);


    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    loiter_nav->update();

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading(loiter_nav->get_thrust_vector(), target_yaw_rate, false);

    // update the vertical offset based on the surface measurement
    copter.surface_tracking.update_surface_offset();

    // Send the commanded climb rate to the position controller
    pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);


    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

#endif

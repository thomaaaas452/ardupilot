#include "Copter.h"

#if MODE_ATLO_ENABLED == ENABLED

/*
 * Init and run calls for loiter flight mode
 */

// loiter_init - initialise loiter controller
bool ModeAttackLoiter::init(bool ignore_checks)
{
        // float target_roll, target_pitch;
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        uint16_t w = 320;
        uint16_t h = 240;

        float roll_out  = 0;
        float pitch_out = 0;
        float angle_max = copter.aparm.angle_max;
        float angle_limit = attitude_control->get_althold_lean_angle_max_cd();
        float threshold_w = w/8;
        float threshold_h = h/6;

        int16_t xtarget = copter.k210.cx;
        int16_t ytarget = copter.k210.cy;

        
        int16_t target_body_frame_y = (int16_t)xtarget - 159;  // QQVGA 320 * 240

        float angle_y_deg = target_body_frame_y * 60.0f / w;

        if(abs(xtarget - 159) < threshold_w)
        {
            roll_out = int(angle_y_deg * g2.atlo_roll);
        } else {
            float roll_mod = angle_y_deg / abs(angle_y_deg);
            roll_out = int(roll_mod * 7.5 * g2.atlo_roll);
        }

        if((abs(ytarget - 120 + threshold_h*0.5) < threshold_h)&&(abs(xtarget - 159) < threshold_w))
        {
            pitch_out = int(g2.atlo_pitch);
        }else{
            pitch_out = 0;
        }

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


// loiter_run - runs the loiter controller
// should be called at 100hz or more
void ModeAttackLoiter::run()
{
    // float target_roll, target_pitch;
    float target_yaw_rate = 0.0f;
    float target_climb_rate = 0.0f;
    static uint32_t change_time_ms = 0;

    int16_t target_x = copter.k210.cx;
    int16_t target_y = copter.k210.cy;

    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    update_simple_mode();
    uint16_t w = 320;
    uint16_t h = 240;

    float rf_right_up ;
    float rf_right_down ;  //右后
    float rf_min ;

    rf_right_up = copter.rangefinder.distance_cm_orient(ROTATION_YAW_45); //右前
    rf_right_down = copter.rangefinder.distance_cm_orient(ROTATION_YAW_135);  //右后
    rf_min = (rf_right_down - rf_right_up> 0.01f) ? rf_right_up:rf_right_down;
    float distance_cm = (int16_t)copter.k210.cz*0.1;

    gcs().send_text(MAV_SEVERITY_CRITICAL,
            "ATLO F:%.1f %d %d",
            distance_cm,
            target_x,
            target_y);

    // if((target_x == 159)&&(target_y == 120))
    // {
    //     copter.set_mode(Mode::Number::SEMIAUTO, ModeReason::MISSION_END);
    // }

    if((target_x != 159)||(target_y != 120))
    {
        change_time_ms = millis();
    } else {
        if(millis()-change_time_ms>500)
        {
            copter.set_mode(Mode::Number::SEMIAUTO, ModeReason::MISSION_END);
        }
    }
    // 考虑是否添加末端直接前冲
    // if(distance_cm <= 30.0){
    // target_x = 159;
    // target_y = 120;
    // }

    int16_t target_body_frame_y = (int16_t)target_x - 159;  // QQVGA 320 * 240
    int16_t target_body_frame_z = (int16_t)target_y - 120;

    float angle_y_deg = target_body_frame_y * 120.0f / w;
    float angle_z_deg = target_body_frame_z * 60.0f / h;

    float roll_out  = 0;
    float pitch_out = 0;
    // target_yaw_rate = 0.0f;
    float target_climb_rate_raw = 0;

    // float angle_max = loiter_nav->get_angle_max_cd(); 
    float angle_limit = attitude_control->get_althold_lean_angle_max_cd();
    float threshold_w = w/8 ;
    float threshold_h = h/6 ;

    if(distance_cm >= 110){
    if(abs(target_x - 159) < 0.5 * threshold_w)
    {
        roll_out = int(angle_y_deg * g2.atlo_roll);
    } else {
        float roll_mod = angle_y_deg / abs(angle_y_deg);
        roll_out = int(roll_mod * 7.5 * g2.atlo_roll);
    }
    // 打印机架
    //if(rf_min <= 60.0)
    // 碳纤机架
    if(rf_min <= 60.0)
    {
        roll_out = -550;
        // target_yaw_rate = -0.1f;
    }
    } else {
        if(abs(target_x - 159) < 0.5 * threshold_w)
        {
            roll_out = int(angle_y_deg * g2.atlo_roll * 0.5);
        } else {
            float roll_mod = angle_y_deg / abs(angle_y_deg);
            roll_out = int(roll_mod * 3.75 * g2.atlo_roll);
        }
    }

    if(abs(target_y - 120) < 0.5*threshold_h) //考虑到前倾时飞机俯仰角对镜头角度误差，设置识别中心点为(160,120+threshold_h*0.5)
    {
        target_climb_rate_raw = int(484 + angle_z_deg * g2.atlo_climb);
    } else {
        float climb_mod = angle_z_deg / abs(angle_z_deg);
        target_climb_rate_raw = int(484 + climb_mod * 10 * g2.atlo_climb);
    }

    // if(distance_cm >= 110){
    // if((target_y - 120  < 0.01f) && (target_y - 120  > -threshold_h) && (abs(target_x - 159) < threshold_w))
    // {
    //     pitch_out = int(g2.atlo_pitch);
    // } else{
    //     pitch_out = 0;
    // }
    // } else {
    // if((abs(target_y - 120 ) < 0.8*threshold_h) && (abs(target_x - 159) < 0.8*threshold_w))
    // {
    //     pitch_out = int(g2.atlo_pitch);
    // } else{
    //     pitch_out = 0;
    // }
    // }

    if((abs(target_y - 120 ) < threshold_h) && (abs(target_x - 159) < threshold_w))
    {
        pitch_out = int(g2.atlo_pitch);
    } else{
        pitch_out = 0;
    }
    
    // if(distance_cm >= 200.0){
    //     pitch_out = 0.5*int(g2.atlo_pitch);
    //     roll_out = 0;
    // }
    // else 
    if(distance_cm <= 19.0){
        pitch_out = 0;
    }

        // convert pilot input to lean angles
        
        // float angle_limit = attitude_control->get_althold_lean_angle_max_cd();
        angle_limit = constrain_float(angle_limit, 1000.0f, loiter_nav->get_angle_max_cd());

        // scale roll and pitch inputs to ANGLE_MAX parameter range
        float scaler = loiter_nav->get_angle_max_cd()/(float)ROLL_PITCH_YAW_INPUT_MAX;
        roll_out *= scaler;
        pitch_out *= scaler;

        // do circular limit
        float total_in = norm(roll_out, pitch_out);
        if (total_in > angle_limit) {
            float ratio = angle_limit / total_in;
            roll_out *= ratio;
            pitch_out *= ratio;
        }

        // do lateral tilt to euler roll conversion
        roll_out = (18000/M_PI) * atanf(cosf(pitch_out*(M_PI/18000))*tanf(roll_out*(M_PI/18000)));

        // get pilot desired climb rate
        // target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

        // process pilot's roll and pitch input
        loiter_nav->set_pilot_desired_acceleration(roll_out, pitch_out);

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(target_climb_rate_raw);
        

        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);



    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        loiter_nav->soften_for_landing();
    }

    // Loiter State Machine

        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        loiter_nav->update();

        // target_yaw_rate = get_pilot_desired_yaw_rate(target_yaw_rate);

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

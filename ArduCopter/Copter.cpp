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

/*
 *  ArduCopter (also known as APM, APM:Copter or just Copter)
 *  Wiki:           copter.ardupilot.org
 *  Creator:        Jason Short
 *  Lead Developer: Randy Mackay
 *  Lead Tester:    Marco Robustini
 *  Based on code and ideas from the Arducopter team: Leonard Hall, Andrew Tridgell, Robert Lefebvre, Pat Hickey, Michael Oborne, Jani Hirvinen,
                                                      Olivier Adler, Kevin Hester, Arthur Benemann, Jonathan Challinger, John Arne Birkeland,
                                                      Jean-Louis Naudin, Mike Smith, and more
 *  Thanks to: Chris Anderson, Jordi Munoz, Jason Short, Doug Weibel, Jose Julio
 *
 *  Special Thanks to contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera       :Auto Compass Declination
 *  Amilcar Lucas       :Camera mount library
 *  Andrew Tridgell     :General development, Mavlink Support
 *  Andy Piper          :Harmonic notch, In-flight FFT, Bi-directional DShot, various drivers
 *  Angel Fernandez     :Alpha testing
 *  AndreasAntonopoulous:GeoFence
 *  Arthur Benemann     :DroidPlanner GCS
 *  Benjamin Pelletier  :Libraries
 *  Bill King           :Single Copter
 *  Christof Schmid     :Alpha testing
 *  Craig Elder         :Release Management, Support
 *  Dani Saez           :V Octo Support
 *  Doug Weibel         :DCM, Libraries, Control law advice
 *  Emile Castelnuovo   :VRBrain port, bug fixes
 *  Gregory Fletcher    :Camera mount orientation math
 *  Guntars             :Arming safety suggestion
 *  HappyKillmore       :Mavlink GCS
 *  Hein Hollander      :Octo Support, Heli Testing
 *  Igor van Airde      :Control Law optimization
 *  Jack Dunkle         :Alpha testing
 *  James Goppert       :Mavlink Support
 *  Jani Hiriven        :Testing feedback
 *  Jean-Louis Naudin   :Auto Landing
 *  John Arne Birkeland :PPM Encoder
 *  Jose Julio          :Stabilization Control laws, MPU6k driver
 *  Julien Dubois       :PosHold flight mode
 *  Julian Oes          :Pixhawk
 *  Jonathan Challinger :Inertial Navigation, CompassMot, Spin-When-Armed
 *  Kevin Hester        :Andropilot GCS
 *  Max Levine          :Tri Support, Graphics
 *  Leonard Hall        :Flight Dynamics, Throttle, Loiter and Navigation Controllers
 *  Marco Robustini     :Lead tester
 *  Michael Oborne      :Mission Planner GCS
 *  Mike Smith          :Pixhawk driver, coding support
 *  Olivier Adler       :PPM Encoder, piezo buzzer
 *  Pat Hickey          :Hardware Abstraction Layer (HAL)
 *  Robert Lefebvre     :Heli Support, Copter LEDs
 *  Roberto Navoni      :Library testing, Porting to VRBrain
 *  Sandro Benigno      :Camera support, MinimOSD
 *  Sandro Tognana      :PosHold flight mode
 *  Sebastian Quilter   :SmartRTL
 *  ..and many more.
 *
 *  Code commit statistics can be found here: https://github.com/ArduPilot/ardupilot/graphs/contributors
 *  Wiki: https://copter.ardupilot.org/
 *
 */

#include "Copter.h"

#define FORCE_VERSION_H_INCLUDE
#include "version.h"
#undef FORCE_VERSION_H_INCLUDE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SCHED_TASK(func, _interval_ticks, _max_time_micros, _prio) SCHED_TASK_CLASS(Copter, &copter, func, _interval_ticks, _max_time_micros, _prio)

/*
  scheduler table - all regular tasks apart from the fast_loop()
  should be listed here.

  All entries in this table must be ordered by priority.

  This table is interleaved with the table in AP_Vehicle to determine
  the order in which tasks are run.  Convenience methods SCHED_TASK
  and SCHED_TASK_CLASS are provided to build entries in this structure:

SCHED_TASK arguments:
 - name of static function to call
 - rate (in Hertz) at which the function should be called
 - expected time (in MicroSeconds) that the function should take to run
 - priority (0 through 255, lower number meaning higher priority)

SCHED_TASK_CLASS arguments:
 - class name of method to be called
 - instance on which to call the method
 - method to call on that instance
 - rate (in Hertz) at which the method should be called
 - expected time (in MicroSeconds) that the method should take to run
 - priority (0 through 255, lower number meaning higher priority)

 */
const AP_Scheduler::Task Copter::scheduler_tasks[] = {
    SCHED_TASK(rc_loop,              100,    130,  3),
    SCHED_TASK(throttle_loop,         50,     75,  6),
    SCHED_TASK_CLASS(AP_GPS,               &copter.gps,                 update,          50, 200,   9),
#if AP_OPTICALFLOW_ENABLED
    SCHED_TASK_CLASS(OpticalFlow,          &copter.optflow,             update,         200, 160,  12),
#endif
    SCHED_TASK(update_batt_compass,   10,    120, 15),
    SCHED_TASK_CLASS(RC_Channels, (RC_Channels*)&copter.g2.rc_channels, read_aux_all,    10,  50,  18),
    SCHED_TASK(arm_motors_check,      10,     50, 21),
#if TOY_MODE_ENABLED == ENABLED
    SCHED_TASK_CLASS(ToyMode,              &copter.g2.toy_mode,         update,          10,  50,  24),
#endif
    SCHED_TASK(auto_disarm_check,     10,     50,  27),
    SCHED_TASK(auto_trim,             10,     75,  30),
#if RANGEFINDER_ENABLED == ENABLED
    SCHED_TASK(read_rangefinder,      20,    100,  33),
#endif
#if HAL_PROXIMITY_ENABLED
    SCHED_TASK_CLASS(AP_Proximity,         &copter.g2.proximity,        update,         200,  50,  36),
#endif
#if BEACON_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Beacon,            &copter.g2.beacon,           update,         400,  50,  39),
#endif
    SCHED_TASK(update_altitude,       10,    100,  42),
    SCHED_TASK(run_nav_updates,       50,    100,  45),
    SCHED_TASK(update_throttle_hover,100,     90,  48),
#if MODE_SMARTRTL_ENABLED == ENABLED
    SCHED_TASK_CLASS(ModeSmartRTL,         &copter.mode_smartrtl,       save_position,    3, 100,  51),
#endif
#if SPRAYER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AC_Sprayer,           &copter.sprayer,               update,         3,  90,  54),
#endif
    SCHED_TASK(three_hz_loop,          3,     75, 57),
    SCHED_TASK_CLASS(AP_ServoRelayEvents,  &copter.ServoRelayEvents,      update_events, 50,  75,  60),
    SCHED_TASK_CLASS(AP_Baro,              &copter.barometer,             accumulate,    50,  90,  63),
#if AC_FENCE == ENABLED
    SCHED_TASK_CLASS(AC_Fence,             &copter.fence,                 update,        10, 100,  66),
#endif
#if PRECISION_LANDING == ENABLED
    SCHED_TASK(update_precland,      400,     50,  69),
#endif
#if FRAME_CONFIG == HELI_FRAME
    SCHED_TASK(check_dynamic_flight,  50,     75,  72),
#endif
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(fourhundred_hz_logging,400,    50,  75),
#endif
    SCHED_TASK_CLASS(AP_Notify,            &copter.notify,              update,          50,  90,  78),
    SCHED_TASK(one_hz_loop,            1,    100,  81),
    SCHED_TASK(update_K210,           400,   100,  82),                 // 单个循环中，更新K210驱动
    SCHED_TASK(update_height2servo,   10,    100,  83),                 // 触发引信函数（包含高度触发、距离触发引信）
    SCHED_TASK(ekf_check,             10,     75,  84),
    SCHED_TASK(check_vibration,       10,     50,  87),
    SCHED_TASK(gpsglitch_check,       10,     50,  90),
#if LANDING_GEAR_ENABLED == ENABLED
    SCHED_TASK(landinggear_update,    10,     75,  93),
#endif
    SCHED_TASK(standby_update,        100,    75,  96),
    SCHED_TASK(lost_vehicle_check,    10,     50,  99),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_receive, 400, 180, 102),
    SCHED_TASK_CLASS(GCS,                  (GCS*)&copter._gcs,          update_send,    400, 550, 105),
#if HAL_MOUNT_ENABLED
    SCHED_TASK_CLASS(AP_Mount,             &copter.camera_mount,        update,          50,  75, 108),
#endif
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera,            &copter.camera,              update,          50,  75, 111),
#endif
#if LOGGING_ENABLED == ENABLED
    SCHED_TASK(ten_hz_logging_loop,   10,    350, 114),
    SCHED_TASK(twentyfive_hz_logging, 25,    110, 117),
    SCHED_TASK_CLASS(AP_Logger,            &copter.logger,              periodic_tasks, 400, 300, 120),
#endif
    SCHED_TASK_CLASS(AP_InertialSensor,    &copter.ins,                 periodic,       400,  50, 123),

    SCHED_TASK_CLASS(AP_Scheduler,         &copter.scheduler,           update_logging, 0.1,  75, 126),
#if RPM_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_RPM,               &copter.rpm_sensor,          update,          40, 200, 129),
#endif
    SCHED_TASK_CLASS(Compass, &copter.compass, cal_update, 100, 100, 132),
    SCHED_TASK_CLASS(AP_TempCalibration,   &copter.g2.temp_calibration, update,          10, 100, 135),
#if HAL_ADSB_ENABLED
    SCHED_TASK(avoidance_adsb_update, 10,    100, 138),
#endif
#if ADVANCED_FAILSAFE == ENABLED
    SCHED_TASK(afs_fs_check,          10,    100, 141),
#endif
#if AP_TERRAIN_AVAILABLE
    SCHED_TASK(terrain_update,        10,    100, 144),
#endif
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper,           &copter.g2.gripper,          update,          10,  75, 147),
#endif
#if WINCH_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Winch,             &copter.g2.winch,            update,          50,  50, 150),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75, 153),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75, 156),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75, 159),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,      3.3,   75, 162),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,     75, 165),
#endif
#if HAL_BUTTON_ENABLED
    SCHED_TASK_CLASS(AP_Button,            &copter.button,              update,           5, 100, 168),
#endif
#if STATS_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Stats,             &copter.g2.stats,            update,           1, 100, 171),
#endif
};

void Copter::get_scheduler_tasks(const AP_Scheduler::Task *&tasks,
                                 uint8_t &task_count,
                                 uint32_t &log_bit)
{
    tasks = &scheduler_tasks[0];
    task_count = ARRAY_SIZE(scheduler_tasks);
    log_bit = MASK_LOG_PM;
}

constexpr int8_t Copter::_failsafe_priorities[7];

// Main loop - 400hz
void Copter::fast_loop()
{
    // update INS immediately to get current gyro data populated
    ins.update();

    // run low level rate controllers that only require IMU data
    attitude_control->rate_controller_run();

    // send outputs to the motors library immediately
    motors_output();

    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS();

#if FRAME_CONFIG == HELI_FRAME
    update_heli_control_dynamics();
    #if MODE_AUTOROTATE_ENABLED == ENABLED
        heli_update_autorotation();
    #endif
#endif //HELI_FRAME

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading or position
    check_ekf_reset();

    // run the attitude controllers
    update_flight_mode();

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've landed or crashed
    update_land_and_crash_detectors();

#if HAL_MOUNT_ENABLED
    // camera mount's fast update
    camera_mount.update_fast();
#endif

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }

    AP_Vehicle::fast_loop();

    if (should_log(MASK_LOG_VIDEO_STABILISATION)) {
        ahrs.write_video_stabilisation();
    }
}

#if AP_SCRIPTING_ENABLED
// start takeoff to given altitude (for use by scripting)
bool Copter::start_takeoff(float alt)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    if (mode_guided.do_user_takeoff_start(alt * 100.0f)) {
        copter.set_auto_armed(true);
        return true;
    }
    return false;
}

// set target location (for use by scripting)
bool Copter::set_target_location(const Location& target_loc)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    return mode_guided.set_destination(target_loc);
}

// set target position (for use by scripting)
bool Copter::set_target_pos_NED(const Vector3f& target_pos, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative, bool terrain_alt)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);

    return mode_guided.set_destination(pos_neu_cm, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, yaw_relative, terrain_alt);
}

// set target position and velocity (for use by scripting)
bool Copter::set_target_posvel_NED(const Vector3f& target_pos, const Vector3f& target_vel)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);

    return mode_guided.set_destination_posvelaccel(pos_neu_cm, vel_neu_cms, Vector3f());
}

// set target position, velocity and acceleration (for use by scripting)
bool Copter::set_target_posvelaccel_NED(const Vector3f& target_pos, const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool yaw_relative)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    const Vector3f pos_neu_cm(target_pos.x * 100.0f, target_pos.y * 100.0f, -target_pos.z * 100.0f);
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);
    const Vector3f accel_neu_cms(target_accel.x * 100.0f, target_accel.y * 100.0f, -target_accel.z * 100.0f);

    return mode_guided.set_destination_posvelaccel(pos_neu_cm, vel_neu_cms, accel_neu_cms, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, yaw_relative);
}

bool Copter::set_target_velocity_NED(const Vector3f& vel_ned)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // convert vector to neu in cm
    const Vector3f vel_neu_cms(vel_ned.x * 100.0f, vel_ned.y * 100.0f, -vel_ned.z * 100.0f);
    mode_guided.set_velocity(vel_neu_cms);
    return true;
}

// set target velocity and acceleration (for use by scripting)
bool Copter::set_target_velaccel_NED(const Vector3f& target_vel, const Vector3f& target_accel, bool use_yaw, float yaw_deg, bool use_yaw_rate, float yaw_rate_degs, bool relative_yaw)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    // convert vector to neu in cm
    const Vector3f vel_neu_cms(target_vel.x * 100.0f, target_vel.y * 100.0f, -target_vel.z * 100.0f);
    const Vector3f accel_neu_cms(target_accel.x * 100.0f, target_accel.y * 100.0f, -target_accel.z * 100.0f);

    mode_guided.set_velaccel(vel_neu_cms, accel_neu_cms, use_yaw, yaw_deg * 100.0, use_yaw_rate, yaw_rate_degs * 100.0, relative_yaw);
    return true;
}

bool Copter::set_target_angle_and_climbrate(float roll_deg, float pitch_deg, float yaw_deg, float climb_rate_ms, bool use_yaw_rate, float yaw_rate_degs)
{
    // exit if vehicle is not in Guided mode or Auto-Guided mode
    if (!flightmode->in_guided_mode()) {
        return false;
    }

    Quaternion q;
    q.from_euler(radians(roll_deg),radians(pitch_deg),radians(yaw_deg));

    mode_guided.set_angle(q, Vector3f{}, climb_rate_ms*100, false);
    return true;
}

// circle mode controls
bool Copter::get_circle_radius(float &radius_m)
{
    radius_m = circle_nav->get_radius() * 0.01f;
    return true;
}

bool Copter::set_circle_rate(float rate_dps)
{
    circle_nav->set_rate(rate_dps);
    return true;
}

// returns true if mode supports NAV_SCRIPT_TIME mission commands
bool Copter::nav_scripting_enable(uint8_t mode)
{
    return mode == (uint8_t)mode_auto.mode_number();
}

// lua scripts use this to retrieve the contents of the active command
bool Copter::nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2)
{
    if (flightmode != &mode_auto) {
        return false;
    }

    return mode_auto.nav_script_time(id, cmd, arg1, arg2);
}

// lua scripts use this to indicate when they have complete the command
void Copter::nav_script_time_done(uint16_t id)
{
    if (flightmode != &mode_auto) {
        return;
    }

    return mode_auto.nav_script_time_done(id);
}

#endif // AP_SCRIPTING_ENABLED


// rc_loops - reads user input from transmitter/receiver
// called at 100hz
void Copter::rc_loop()
{
    // Read radio and 3-position switch on radio
    // -----------------------------------------
    read_radio();
    rc().read_mode_switch();
}

// throttle_loop - should be run at 50 hz
// ---------------------------
void Copter::throttle_loop()
{
    // update throttle_low_comp value (controls priority of throttle vs attitude control)
    update_throttle_mix();

    // check auto_armed status
    update_auto_armed();

#if FRAME_CONFIG == HELI_FRAME
    // update rotor speed
    heli_update_rotor_speed_targets();

    // update trad heli swash plate movement
    heli_update_landing_swash();
#endif

    // compensate for ground effect (if enabled)
    update_ground_effect_detector();
    update_ekf_terrain_height_stable();
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Copter::update_batt_compass(void)
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if(AP::compass().available()) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors->get_throttle());
        compass.set_voltage(battery.voltage());
        compass.read();
    }
}

// 更新K210驱动
void Copter::update_K210(void)
{
    k210.update();
}

// 触发引信流程函数
// 特征信号为PWM方波，占空比为10%，持续时间至少0.4s，需要注意的是在不输出特征信号时，输出特征信号的通道上同样需要输出指定占空比为0的PWM方波。
// 引信触发包含解保、起爆两个环节；
// 解保环节需要两次解保，每次解保需要输出特征信号至引信板，两次解保后进入起爆阶段；起爆分为距离起爆和高度起爆，需要提前设定模式（g2.yinxin_channel）
// 起爆模式同样需要两次特征信号；
// 距离起爆: 在拨杆或进入自主攻击模式时发出第一次特征信号；在距离达到设定值时（目前设定的是20cm）发出第二次特征信号，完成起爆。
// 高度起爆: 在拨杆后进入高度起爆，到达设定的初步高度后（目前设定的是90-150cm）发出第一次特征信号；到达设定的最终高度后（目前设定的是110-130cm）发出第二次特征信号，完成起爆。
void Copter::update_height2servo(void)
{

    float distance_front_cm = (int16_t)(k210.cz*0.1); //前向距离
    float distance_down_cm = rangefinder.distance_cm_orient(ROTATION_PITCH_270); // 下向距离 
    static uint8_t step = 0;  //步骤
    static uint32_t jiebao_time_ms = 0; // 解保时间
    // bool jiebao_success = false;
    static uint32_t qibao_time_ms = 0; // 起爆时间

    uint16_t rcin_jiebao_first = hal.rcin->read(8); //第一次解保输入信号为遥控器的第7通道
    uint16_t rcin_jiebao_second = hal.rcin->read(9); //第二次解保输入信号为遥控器的第8通道
    uint16_t rcin_gaodu = hal.rcin->read(6); //遥控器的第5通道为选择高度触发模式
    uint16_t rcin_juli  = hal.rcin->read(7); //遥控器的第6通道为选择前向距离触发模式

    SRV_Channel::Aux_servo_function_t function_jiebao = SRV_Channels::get_motor_function(4); //设置电（舵）机通道5作为解保信号输出
    SRV_Channel::Aux_servo_function_t function_qibao = SRV_Channels::get_motor_function(5);  //设置电（舵）机通道6作为起爆信号输出

    SRV_Channels::set_output_pwm(function_jiebao, 0); // 设置解保输出通道值为0 
    SRV_Channels::set_output_pwm(function_qibao, 0);  // 设置起爆输出通道值为0

    if(g2.yinxin_out != 0) //yinxin_out为引信使能参数，0为不使用引信，1为使用引信
    {
        switch(step){
        case 0:
            if(rcin_jiebao_first < 1800) // 指rcin_jiebao_first不拨杆时（位于中低位）
            {
                jiebao_time_ms = millis();
                SRV_Channels::set_output_pwm(function_jiebao, 0); //此时第一次解保通道输出PWM占空比为0
            }
            else if(rcin_jiebao_first > 1800) // rcin_jiebao_first拨杆（位于高位）
            {
                //在0.5s内连续发出
                if(millis() - jiebao_time_ms < 500){
                    SRV_Channels::set_output_pwm(function_jiebao, 2000);  //此时第一次解保通道输出PWM占空比为10%
                }
                else{
                    SRV_Channels::set_output_pwm(function_jiebao, 0);   //此时第一次解保通道输出PWM占空比为0%
                    step = 1; //这样就完成了第一次解保信号的发出
                }
            }
            break;

        case 1:
            if(rcin_jiebao_second < 1800) // 指rcin_jiebao_second不拨杆时（位于中低位）
            {
                jiebao_time_ms = millis();
                SRV_Channels::set_output_pwm(function_jiebao, 0); //此时第二次解保通道输出PWM占空比为0
            }
            else if(rcin_jiebao_second > 1800) // rcin_jiebao_second拨杆（位于高位）
            {
                //在0.5s内连续发出
                if(millis() - jiebao_time_ms < 500){
                    SRV_Channels::set_output_pwm(function_jiebao, 2000); //此时第二次解保通道输出PWM占空比为10%
                }
                else{
                    SRV_Channels::set_output_pwm(function_jiebao, 0);  //此时第二次解保通道输出PWM占空比为0%
                    step = 2; //这样就完成了第二次解保信号的发出，此时完成引信全部解保
                }
            }
            break;

        case 2:
            if(g2.yinxin_channel == 1) //yinxin_channel为引信起爆模式选择，1为前向距离触发
            {
                //当距离通道输出不拨杆（位于中高位），或飞行模式不为攻击模式（即，飞行模式为攻击模式时或拨杆至低位时，执行下面程序）
                if((rcin_juli > 1200)&&(copter.flightmode->mode_number() != Mode::Number::ATLO)) 
                {
                    qibao_time_ms = millis();
                    SRV_Channels::set_output_pwm(function_qibao, 0);  //此时起爆通道输出PWM占空比为0
                }
                else{
                    //在0.5s内连续发出
                    if(millis() - qibao_time_ms < 500){ 
                        SRV_Channels::set_output_pwm(function_qibao, 2000);  //此时起爆通道输出PWM占空比为10%（起爆第一次信号发出）
                    }
                    else{
                        SRV_Channels::set_output_pwm(function_qibao, 0); //此时起爆通道输出PWM占空比为10%（起爆第一次信号结束）
                        step = 3;
                    }
                }
                // break;  
            }

            else if(g2.yinxin_channel == 2) //yinxin_channel为引信起爆模式选择，2为高度距离触发
            {
                //当高度通道输出不拨杆（位于高位）（即，拨杆至中低位时，执行下面程序）
                if(rcin_gaodu > 1800){ 
                    step = 3;
                }

            }
            break;

        case 3:
            if(g2.yinxin_channel == 1) //前向距离触发
            {
                //前向距离小于等于20cm时，触发起爆第二次信号，并完成引爆
                if(distance_front_cm >= 20) 
                {
                    //6月4日晚上添加
                    qibao_time_ms = millis();
                    //
                    SRV_Channels::set_output_pwm(function_qibao, 0); 
                }
                else{
                    //6月4日晚上添加
                    if(millis() - qibao_time_ms < 500){
                        SRV_Channels::set_output_pwm(function_qibao, 2000); 
                    }
                    else{
                        SRV_Channels::set_output_pwm(function_qibao, 0); 
                    }
                    //
                    // SRV_Channels::set_output_pwm(function_qibao, 2000); 
                }
                // break;
            }

            if(g2.yinxin_channel == 2) //高度距离触发
            {
                // 当距离地面高度为90cm-150cm时，起爆第一次信号发出
                if((distance_down_cm>150)||(distance_down_cm<90))
                {
                    qibao_time_ms = millis();
                    SRV_Channels::set_output_pwm(function_qibao, 0); 
                }
                else{
                    // 持续0.5s
                    if(millis() - qibao_time_ms < 500){
                        SRV_Channels::set_output_pwm(function_qibao, 2000); 
                    }
                    else{
                        SRV_Channels::set_output_pwm(function_qibao, 0); 
                        step = 4;
                    }
                }
                // break;
            }
            break;

        case 4:
            if(g2.yinxin_channel == 2)//高度距离触发
            {
                // 当距离地面高度为110cm-130cm时，起爆第一次信号发出
                if((distance_down_cm > 130)||(distance_down_cm < 110))
                {
                    qibao_time_ms = millis();
                    SRV_Channels::set_output_pwm(function_qibao, 0); 
                }
                else{
                    // 持续0.5s
                    if(millis() - qibao_time_ms < 500){
                        SRV_Channels::set_output_pwm(function_qibao, 2000);  
                    }
                    else{
                        SRV_Channels::set_output_pwm(function_qibao, 0); 
                        step = 5;
                    }
                }
                // break;
            }
            break;    
        }
    }

}

// Full rate logging of attitude, rate and pid loops
// should be run at 400hz
void Copter::fourhundred_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        Log_Write_Attitude();
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Copter::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST) && !copter.flightmode->logs_attitude()) {
        Log_Write_Attitude();
    }
    // log EKF attitude data
    if (should_log(MASK_LOG_ATTITUDE_MED) || should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        motors->Log_Write();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
        if (rssi.enabled()) {
            logger.Write_RSSI();
        }
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && (flightmode->requires_GPS() || landing_with_GPS() || !flightmode->has_manual_throttle())) {
        pos_control->write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        AP::ins().Write_Vibration();
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control->control_monitor_log();
#if HAL_PROXIMITY_ENABLED
        logger.Write_Proximity(g2.proximity);  // Write proximity sensor distances
#endif
#if BEACON_ENABLED == ENABLED
        logger.Write_Beacon(g2.beacon);
#endif
    }
#if FRAME_CONFIG == HELI_FRAME
    Log_Write_Heli();
#endif
#if WINCH_ENABLED == ENABLED
    if (should_log(MASK_LOG_ANY)) {
        g2.winch.write_log();
    }
#endif
}

// twentyfive_hz_logging - should be run at 25hz
void Copter::twentyfive_hz_logging()
{
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_EKF_POS();
    }

    if (should_log(MASK_LOG_IMU)) {
        AP::ins().Write_IMU();
    }

#if MODE_AUTOROTATE_ENABLED == ENABLED
    if (should_log(MASK_LOG_ATTITUDE_MED) || should_log(MASK_LOG_ATTITUDE_FAST)) {
        //update autorotation log
        g2.arot.Log_Write_Autorotation();
    }
#endif
}

// three_hz_loop - 3.3hz loop
void Copter::three_hz_loop()
{
    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED


    // update ch6 in flight tuning
    tuning();

    // check if avoidance should be enabled based on alt
    low_alt_avoidance();
}

// one_hz_loop - runs at 1Hz
void Copter::one_hz_loop()
{
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap.value);
    }

    arming.update();

    if (!motors->armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.update_orientation();

        update_using_interlock();

        // check the user hasn't updated the frame class or type
        motors->set_frame_class_and_type((AP_Motors::motor_frame_class)g2.frame_class.get(), (AP_Motors::motor_frame_type)g.frame_type.get());

#if FRAME_CONFIG != HELI_FRAME
        // set all throttle channel settings
        motors->update_throttle_range();
#endif
    }

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    // log terrain data
    terrain_logging();

#if HAL_ADSB_ENABLED
    adsb.set_is_flying(!ap.land_complete);
#endif

    AP_Notify::flags.flying = !ap.land_complete;

    gcs().send_text(MAV_SEVERITY_CRITICAL,
                "F:%.1f %d %d",
                k210.cz/10.0f,
                k210.cx,
                k210.cy
                );
    


}

void Copter::init_simple_bearing()
{
    // capture current cos_yaw and sin_yaw values
    simple_cos_yaw = ahrs.cos_yaw();
    simple_sin_yaw = ahrs.sin_yaw();

    // initialise super simple heading (i.e. heading towards home) to be 180 deg from simple mode heading
    super_simple_last_bearing = wrap_360_cd(ahrs.yaw_sensor+18000);
    super_simple_cos_yaw = simple_cos_yaw;
    super_simple_sin_yaw = simple_sin_yaw;

    // log the simple bearing
    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::INIT_SIMPLE_BEARING, ahrs.yaw_sensor);
    }
}

// update_simple_mode - rotates pilot input if we are in simple mode
void Copter::update_simple_mode(void)
{
    float rollx, pitchx;

    // exit immediately if no new radio frame or not in simple mode
    if (simple_mode == SimpleMode::NONE || !ap.new_radio_frame) {
        return;
    }

    // mark radio frame as consumed
    ap.new_radio_frame = false;

    if (simple_mode == SimpleMode::SIMPLE) {
        // rotate roll, pitch input by -initial simple heading (i.e. north facing)
        rollx = channel_roll->get_control_in()*simple_cos_yaw - channel_pitch->get_control_in()*simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*simple_sin_yaw + channel_pitch->get_control_in()*simple_cos_yaw;
    }else{
        // rotate roll, pitch input by -super simple heading (reverse of heading to home)
        rollx = channel_roll->get_control_in()*super_simple_cos_yaw - channel_pitch->get_control_in()*super_simple_sin_yaw;
        pitchx = channel_roll->get_control_in()*super_simple_sin_yaw + channel_pitch->get_control_in()*super_simple_cos_yaw;
    }

    // rotate roll, pitch input from north facing to vehicle's perspective
    channel_roll->set_control_in(rollx*ahrs.cos_yaw() + pitchx*ahrs.sin_yaw());
    channel_pitch->set_control_in(-rollx*ahrs.sin_yaw() + pitchx*ahrs.cos_yaw());
}

// update_super_simple_bearing - adjusts simple bearing based on location
// should be called after home_bearing has been updated
void Copter::update_super_simple_bearing(bool force_update)
{
    if (!force_update) {
        if (simple_mode != SimpleMode::SUPERSIMPLE) {
            return;
        }
        if (home_distance() < SUPER_SIMPLE_RADIUS) {
            return;
        }
    }

    const int32_t bearing = home_bearing();

    // check the bearing to home has changed by at least 5 degrees
    if (labs(super_simple_last_bearing - bearing) < 500) {
        return;
    }

    super_simple_last_bearing = bearing;
    const float angle_rad = radians((super_simple_last_bearing+18000)/100);
    super_simple_cos_yaw = cosf(angle_rad);
    super_simple_sin_yaw = sinf(angle_rad);
}

void Copter::read_AHRS(void)
{
    // we tell AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
}

// read baro and log control tuning
void Copter::update_altitude()
{
    // read in baro altitude
    read_barometer();

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
        AP::ins().write_notch_log_messages();
#if HAL_GYROFFT_ENABLED
        gyro_fft.write_log_messages();
#endif
    }
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_distance_m(float &distance) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    distance = flightmode->wp_distance() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_bearing_deg(float &bearing) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    bearing = flightmode->wp_bearing() * 0.01;
    return true;
}

// vehicle specific waypoint info helpers
bool Copter::get_wp_crosstrack_error_m(float &xtrack_error) const
{
    // see GCS_MAVLINK_Copter::send_nav_controller_output()
    xtrack_error = flightmode->crosstrack_error() * 0.01;
    return true;
}

/*
  constructor for main Copter class
 */
Copter::Copter(void)
    : logger(g.log_bitmask),
    flight_modes(&g.flight_mode1),
    simple_cos_yaw(1.0f),
    super_simple_cos_yaw(1.0),
    land_accel_ef_filter(LAND_DETECTOR_ACCEL_LPF_CUTOFF),
    rc_throttle_control_in_filter(1.0f),
    inertial_nav(ahrs),
    param_loader(var_info),
    flightmode(&mode_stabilize)
{
    // init sensor error logging flags
    sensor_health.baro = true;
    sensor_health.compass = true;
}

Copter copter;
AP_Vehicle& vehicle = copter;

AP_HAL_MAIN_CALLBACKS(&copter);

#include "Plane.h"
#include <A_statefeedback/my.h>
// #include <A_statefeedback/kalman.h>
// #include <AP_Math/Vector3.h>        // ArduPilot Mega Vector/Matrix math Library
/*
  calculate speed scaling number for control surfaces. This is applied
  to PIDs to change the scaling of the PID with speed. At high speed
  we move the surfaces less, and at low speeds we move them more.
 */
float Plane::calc_speed_scaler(void)
{
    float aspeed, speed_scaler;
    if (ahrs.airspeed_estimate(aspeed)) {
        if (aspeed > auto_state.highest_airspeed && arming.is_armed_and_safety_off()) {
            auto_state.highest_airspeed = aspeed;
        }
        // ensure we have scaling over the full configured airspeed
        const float airspeed_min = MAX(aparm.airspeed_min, MIN_AIRSPEED_MIN);
        const float scale_min = MIN(0.5, g.scaling_speed / (2.0 * aparm.airspeed_max));
        const float scale_max = MAX(2.0, g.scaling_speed / (0.7 * airspeed_min));
        if (aspeed > 0.0001f) {
            speed_scaler = g.scaling_speed / aspeed;
        } else {
            speed_scaler = scale_max;
        }
        speed_scaler = constrain_float(speed_scaler, scale_min, scale_max);

#if HAL_QUADPLANE_ENABLED
        if (quadplane.in_vtol_mode() && arming.is_armed_and_safety_off()) {
            // when in VTOL modes limit surface movement at low speed to prevent instability
            float threshold = airspeed_min * 0.5;
            if (aspeed < threshold) {
                float new_scaler = linear_interpolate(0.001, g.scaling_speed / threshold, aspeed, 0, threshold);
                speed_scaler = MIN(speed_scaler, new_scaler);

                // we also decay the integrator to prevent an integrator from before
                // we were at low speed persistent at high speed
                rollController.decay_I();
                pitchController.decay_I();
                yawController.decay_I();
            }
        }
#endif
    } else if (arming.is_armed_and_safety_off()) {
        // scale assumed surface movement using throttle output
        float throttle_out = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), 1);
        speed_scaler = sqrtf(THROTTLE_CRUISE / throttle_out);
        // This case is constrained tighter as we don't have real speed info
        speed_scaler = constrain_float(speed_scaler, 0.6f, 1.67f);
    } else {
        // no speed estimate and not armed, use a unit scaling
        speed_scaler = 1;
    }
    if (!plane.ahrs.using_airspeed_sensor()  && 
        (plane.flight_option_enabled(FlightOptions::SURPRESS_TKOFF_SCALING)) &&
        (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF)) { //scaling is suppressed during climb phase of automatic takeoffs with no airspeed sensor being used due to problems with inaccurate airspeed estimates
        return MIN(speed_scaler, 1.0f) ;
    }
    return speed_scaler;
}

/*
  return true if the current settings and mode should allow for stick mixing
 */
bool Plane::stick_mixing_enabled(void)
{
    if (!rc().has_valid_input()) {
        // never stick mix without valid RC
        return false;
    }
#if AP_FENCE_ENABLED
    const bool stickmixing = fence_stickmixing();
#else
    const bool stickmixing = true;
#endif
#if HAL_QUADPLANE_ENABLED
    if (control_mode == &mode_qrtl &&
        quadplane.poscontrol.get_state() >= QuadPlane::QPOS_POSITION1) {
        // user may be repositioning
        return false;
    }
    if (quadplane.in_vtol_land_poscontrol()) {
        // user may be repositioning
        return false;
    }
#endif
    if (control_mode->does_auto_throttle() && plane.control_mode->does_auto_navigation()) {
        // we're in an auto mode. Check the stick mixing flag
        if (g.stick_mixing != StickMixing::NONE &&
            g.stick_mixing != StickMixing::VTOL_YAW &&
            stickmixing) {
            return true;
        } else {
            return false;
        }
    }

    if (failsafe.rc_failsafe && g.fs_action_short == FS_ACTION_SHORT_FBWA) {
        // don't do stick mixing in FBWA glide mode
        return false;
    }

    // non-auto mode. Always do stick mixing
    return true;
}


/*
  this is the main roll stabilization function. It takes the
  previously set nav_roll calculates roll servo_out to try to
  stabilize the plane at the given roll
 */
void Plane::stabilize_roll()
{
    if (fly_inverted()) {
        // we want to fly upside down. We need to cope with wrap of
        // the roll_sensor interfering with wrap of nav_roll, which
        // would really confuse the PID code. The easiest way to
        // handle this is to ensure both go in the same direction from
        // zero
        nav_roll_cd += 18000;
        if (ahrs.roll_sensor < 0) nav_roll_cd -= 36000;
    }

    const float roll_out = stabilize_roll_get_roll_out();
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_out);
}








void Plane::stabilize_roll_my()
{
    if (fly_inverted()) {
        // we want to fly upside down. We need to cope with wrap of
        // the roll_sensor interfering with wrap of nav_roll, which
        // would really confuse the PID code. The easiest way to
        // handle this is to ensure both go in the same direction from
        // zero
        nav_roll_cd += 18000;
        if (ahrs.roll_sensor < 0) nav_roll_cd -= 36000;
    }

    const float roll_out = stabilize_roll_get_roll_out_my();
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_out);
}














float Plane::stabilize_roll_get_roll_out()
{
    const float speed_scaler = get_speed_scaler();//重要,TECS需要 导航cpp中计算
    // hal.console -> printf("%f\n",speed_scaler);
#if HAL_QUADPLANE_ENABLED
    if (!quadplane.use_fw_attitude_controllers()) {
        // use the VTOL rate for control, to ensure consistency
        const auto &pid_info = quadplane.attitude_control->get_rate_roll_pid().get_pid_info();

        // scale FF to angle P
        if (quadplane.option_is_set(QuadPlane::OPTION::SCALE_FF_ANGLE_P)) {
            const float mc_angR = quadplane.attitude_control->get_angle_roll_p().kP()
                * quadplane.attitude_control->get_last_angle_P_scale().x;
            if (is_positive(mc_angR)) {
                rollController.set_ff_scale(MIN(1.0, 1.0 / (mc_angR * rollController.tau())));
            }
        }

        const float roll_out = rollController.get_rate_out(degrees(pid_info.target), speed_scaler);
        /* when slaving fixed wing control to VTOL control we need to decay the integrator to prevent
           opposing integrators balancing between the two controllers
        */
        rollController.decay_I();
        return roll_out;
    }
#endif

    bool disable_integrator = false;
    if (control_mode == &mode_stabilize && channel_roll->get_control_in() != 0) {
        disable_integrator = true;
    }
    return rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, speed_scaler, disable_integrator,
                                        ground_mode && !(plane.flight_option_enabled(FlightOptions::DISABLE_GROUND_PID_SUPPRESSION)));
}











float Plane::stabilize_roll_get_roll_out_my()
{
    const float speed_scaler = get_speed_scaler();

    bool disable_integrator = false;
    if (control_mode == &mode_stabilize && channel_roll->get_control_in() != 0) {
        disable_integrator = true;
    }
    return rollController.get_servo_out_my(nav_roll_cd - ahrs.roll_sensor, speed_scaler, disable_integrator,
                                        ground_mode && !(plane.flight_option_enabled(FlightOptions::DISABLE_GROUND_PID_SUPPRESSION)));
}















/*
  this is the main pitch stabilization function. It takes the
  previously set nav_pitch and calculates servo_out values to try to
  stabilize the plane at the given attitude.
 */
void Plane::stabilize_pitch()
{
    int8_t force_elevator = takeoff_tail_hold();
    if (force_elevator != 0) {
        // we are holding the tail down during takeoff. Just convert
        // from a percentage to a -4500..4500 centidegree angle
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 45*force_elevator);
        return;
    }

    const float pitch_out = stabilize_pitch_get_pitch_out();
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_out);
}







void Plane::stabilize_pitch_my()
{
    int8_t force_elevator = takeoff_tail_hold();
    if (force_elevator != 0) {
        // we are holding the tail down during takeoff. Just convert
        // from a percentage to a -4500..4500 centidegree angle
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 45*force_elevator);
        return;
    }

    const float pitch_out = stabilize_pitch_get_pitch_out_my();
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_out);
}










float Plane::stabilize_pitch_get_pitch_out()
{
    const float speed_scaler = get_speed_scaler();
#if HAL_QUADPLANE_ENABLED
    if (!quadplane.use_fw_attitude_controllers()) {
        // use the VTOL rate for control, to ensure consistency
        const auto &pid_info = quadplane.attitude_control->get_rate_pitch_pid().get_pid_info();

        // scale FF to angle P
        if (quadplane.option_is_set(QuadPlane::OPTION::SCALE_FF_ANGLE_P)) {
            const float mc_angP = quadplane.attitude_control->get_angle_pitch_p().kP()
                * quadplane.attitude_control->get_last_angle_P_scale().y;
            if (is_positive(mc_angP)) {
                pitchController.set_ff_scale(MIN(1.0, 1.0 / (mc_angP * pitchController.tau())));
            }
        }

        const int32_t pitch_out = pitchController.get_rate_out(degrees(pid_info.target), speed_scaler);
        /* when slaving fixed wing control to VTOL control we need to decay the integrator to prevent
           opposing integrators balancing between the two controllers
        */
        pitchController.decay_I();
        return pitch_out;
    }
#endif
    // if LANDING_FLARE RCx_OPTION switch is set and in FW mode, manual throttle,throttle idle then set pitch to LAND_PITCH_DEG if flight option FORCE_FLARE_ATTITUDE is set
#if HAL_QUADPLANE_ENABLED
    const bool quadplane_in_frwd_transition = quadplane.in_frwd_transition();
#else
    const bool quadplane_in_frwd_transition = false;
#endif

    int32_t demanded_pitch = nav_pitch_cd + int32_t(g.pitch_trim * 100.0) + SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * g.kff_throttle_to_pitch;
    bool disable_integrator = false;
    if (control_mode == &mode_stabilize && channel_pitch->get_control_in() != 0) {
        disable_integrator = true;
    }
    /* force landing pitch if:
       - flare switch high
       - throttle stick at zero thrust
       - in fixed wing non auto-throttle mode
    */
    if (!quadplane_in_frwd_transition &&
        !control_mode->is_vtol_mode() &&
        !control_mode->does_auto_throttle() &&
        flare_mode == FlareMode::ENABLED_PITCH_TARGET &&
        throttle_at_zero()) {
        demanded_pitch = landing.get_pitch_cd();
    }

    // hal.console -> printf("%d\n",demanded_pitch);
    return pitchController.get_servo_out(demanded_pitch - ahrs.pitch_sensor, speed_scaler, disable_integrator,
                                         ground_mode && !(plane.flight_option_enabled(FlightOptions::DISABLE_GROUND_PID_SUPPRESSION)));

}














float Plane::stabilize_pitch_get_pitch_out_my()
{
    const float speed_scaler = get_speed_scaler();


    int32_t demanded_pitch = nav_pitch_cd + int32_t(g.pitch_trim * 100.0) + SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * g.kff_throttle_to_pitch;
    bool disable_integrator = false;
    if (control_mode == &mode_stabilize && channel_pitch->get_control_in() != 0) {
        disable_integrator = true;
    }
    /* force landing pitch if:
       - flare switch high
       - throttle stick at zero thrust
       - in fixed wing non auto-throttle mode
    */
    if (!control_mode->is_vtol_mode() &&
        !control_mode->does_auto_throttle() &&
        flare_mode == FlareMode::ENABLED_PITCH_TARGET &&
        throttle_at_zero()) {
        demanded_pitch = landing.get_pitch_cd();
    }

    return pitchController.get_servo_out_my(demanded_pitch - ahrs.pitch_sensor, speed_scaler, disable_integrator,
                                         ground_mode && !(plane.flight_option_enabled(FlightOptions::DISABLE_GROUND_PID_SUPPRESSION)));
}





















/*
  this gives the user control of the aircraft in stabilization modes, only used in Stabilize Mode
  to be moved to mode_stabilize.cpp in future
 */
void ModeStabilize::stabilize_stick_mixing_direct()
{
    if (!plane.stick_mixing_enabled()) {
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (!plane.quadplane.allow_stick_mixing()) {
        return;
    }
#endif
    float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    aileron = plane.channel_roll->stick_mixing(aileron);
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);

    float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
    elevator = plane.channel_pitch->stick_mixing(elevator);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
}

/*
  this gives the user control of the aircraft in stabilization modes
  using FBW style controls
 */
void Plane::stabilize_stick_mixing_fbw()
{
    if (!stick_mixing_enabled() ||
        control_mode == &mode_acro ||
        control_mode == &mode_fbwa ||
        control_mode == &mode_autotune ||
        control_mode == &mode_fbwb ||
        control_mode == &mode_cruise ||
#if HAL_QUADPLANE_ENABLED
        control_mode == &mode_qstabilize ||
        control_mode == &mode_qhover ||
        control_mode == &mode_qloiter ||
        control_mode == &mode_qland ||
        control_mode == &mode_qacro ||
#if QAUTOTUNE_ENABLED
        control_mode == &mode_qautotune ||
#endif
        !quadplane.allow_stick_mixing() ||
#endif  // HAL_QUADPLANE_ENABLED
        control_mode == &mode_training) {
        return;
    }
    // do FBW style stick mixing. We don't treat it linearly
    // however. For inputs up to half the maximum, we use linear
    // addition to the nav_roll and nav_pitch. Above that it goes
    // non-linear and ends up as 2x the maximum, to ensure that
    // the user can direct the plane in any direction with stick
    // mixing.
    float roll_input = channel_roll->norm_input_dz();
    if (roll_input > 0.5f) {
        roll_input = (3*roll_input - 1);
    } else if (roll_input < -0.5f) {
        roll_input = (3*roll_input + 1);
    }
    nav_roll_cd += roll_input * roll_limit_cd;
    nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);

    if ((control_mode == &mode_loiter) && (plane.flight_option_enabled(FlightOptions::ENABLE_LOITER_ALT_CONTROL))) {
        // loiter is using altitude control based on the pitch stick, don't use it again here
        return;
    }

    float pitch_input = channel_pitch->norm_input_dz();
    if (pitch_input > 0.5f) {
        pitch_input = (3*pitch_input - 1);
    } else if (pitch_input < -0.5f) {
        pitch_input = (3*pitch_input + 1);
    }
    if (fly_inverted()) {
        pitch_input = -pitch_input;
    }
    if (pitch_input > 0) {
        nav_pitch_cd += pitch_input * aparm.pitch_limit_max*100;
    } else {
        nav_pitch_cd += -(pitch_input * pitch_limit_min*100);
    }
    nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min*100, aparm.pitch_limit_max.get()*100);
}


/*
  stabilize the yaw axis. There are 3 modes of operation:

    - hold a specific heading with ground steering
    - rate controlled with ground steering
    - yaw control for coordinated flight    
 */
void Plane::stabilize_yaw()
{
    bool ground_steering = false;
    if (landing.is_flaring()) {
        // in flaring then enable ground steering
        ground_steering = true;
    } else {
        // otherwise use ground steering when no input control and we
        // are below the GROUND_STEER_ALT
        ground_steering = (channel_roll->get_control_in() == 0 && 
                                            fabsf(relative_altitude) < g.ground_steer_alt);
        if (!landing.is_ground_steering_allowed()) {
            // don't use ground steering on landing approach
            ground_steering = false;
        }
    }


    /*
      first calculate steering for a nose or tail
      wheel. We use "course hold" mode for the rudder when either performing
      a flare (when the wings are held level) or when in course hold in
      FBWA mode (when we are below GROUND_STEER_ALT)
     */
    float steering_output = 0.0;
    if (landing.is_flaring() ||
        (steer_state.hold_course_cd != -1 && ground_steering)) {
        steering_output = calc_nav_yaw_course();
    } else if (ground_steering) {
        steering_output = calc_nav_yaw_ground();
    }

    /*
      now calculate rudder for the rudder
     */
    const float rudder_output = calc_nav_yaw_coordinated();

    if (!ground_steering) {
        // Not doing ground steering, output rudder on steering channel
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder_output);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder_output);

    } else if (!SRV_Channels::function_assigned(SRV_Channel::k_steering)) {
        // Ground steering active but no steering output configured, output steering on rudder channel
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, steering_output);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering_output);

    } else {
        // Ground steering with both steering and rudder channels
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder_output);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, steering_output);
    }

}















void Plane::stabilize_rpy()
{
    //滚转
    if (fly_inverted()) {
        // we want to fly upside down. We need to cope with wrap of
        // the roll_sensor interfering with wrap of nav_roll, which
        // would really confuse the PID code. The easiest way to
        // handle this is to ensure both go in the same direction from
        // zero
        nav_roll_cd += 18000;
        if (ahrs.roll_sensor < 0) nav_roll_cd -= 36000;
    }

    //废弃方案
    // const float pdot = stabilize_roll_get_roll_out_my();
    // const float qdot = stabilize_pitch_get_pitch_out_my();
    // const Vector3<float> rpy_out = stabilize_get_rpy_out(pdot,qdot);
    // const float roll = stabilize_roll_get_roll_out_my();
    // const float pitch = stabilize_pitch_get_pitch_out_my();    
    // const Vector3<float> rpy_out = stabilize_get_rpy_out_2(roll,pitch);

    const Vector3<float> rpy_out = stabilize_get_rpy_out();
    const float roll_out = rpy_out.x;
    const float pitch_out = rpy_out.y;


    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_out);
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_out);



}

Vector3<float> Plane::stabilize_get_rpy_out()
{

    const float dt = AP::scheduler().get_loop_period_s();
    const AP_AHRS &_ahrs = AP::ahrs();

    float aspeed;
    //低空速修正
    if (!_ahrs.airspeed_estimate(aspeed)) {
        aspeed = 0;
    }
    bool underspeed = aspeed <= 7.0f;
    float rate_y = _ahrs.get_gyro().y;
    float rate_x = _ahrs.get_gyro().x;
    float rate_z = _ahrs.get_gyro().z;


    //这些限制条件根据不同飞机性能来确定   需要修改导航模式的滚转俯仰先后顺序
    //以下滚转限制
    // float demanded_roll =radians((nav_roll_cd- ahrs.roll_sensor)*0.01); 
    float demanded_roll =radians(nav_roll_cd*0.01)-ahrs.get_roll();   

    // 限制滚转速率,普通固定翼飞机，最大滚转速率通常在 45°/s 到 120°/s 的范围内
    if (demanded_roll < -radians(10.0f)) {
        demanded_roll = - radians(10.0f);
    } else if (demanded_roll > radians(10.0f)) {
        demanded_roll = radians(10.0f);
    } 
    //以上滚转限制


    //以下俯仰限制 起飞时候稍微推一点点的杆 不要出现抬头趋势，逐渐达到稳态再抬头
    //float demanded_pitch = radians((nav_pitch_cd- ahrs.pitch_sensor)*0.01);// + int32_t(g.pitch_trim * 100.0) + SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * g.kff_throttle_to_pitch;
    float demanded_pitch = radians(nav_pitch_cd*0.01) - ahrs.get_pitch();

    float rate_offset;
    bool inverted = 0;//假设正常飞行

    rate_offset = radians( _get_coordination_rate_offset_my(aspeed, inverted));

    // 限制最大俯仰率需求。倒飞时不应用此限制，
    // 因为当飞行器恢复正常姿态时，俯仰率会进行调节，
    // 并且翻转时通常需要更高的俯仰率。
    if (!inverted) {
        demanded_pitch += rate_offset;
    } else {
        // Make sure not to invert the turn coordination offset
        demanded_pitch = -demanded_pitch + rate_offset;
    }

    /*
    当飞机的滚转角超过用户定义的滚转限制时，我们的优先任务是将飞机的滚转角恢复到限制范围内。
    在较大的滚转角下使用升降舵进行俯仰控制是低效的，并且可能适得其反，因为这样会引入地球坐标系下的偏航，从而降低滚转控制的能力。
    因此，当滚转角超过配置的限制时，我们会线性减少所需的俯仰速率需求，并在90度时将其降低为零。
    */
    float roll_wrapped = labs(_ahrs.roll_sensor);
    if (roll_wrapped > 9000) {
        roll_wrapped = 18000 - roll_wrapped;
    }
    const float roll_limit_margin = 3000;
    if (roll_wrapped > roll_limit_margin && labs(_ahrs.pitch_sensor) < 3000) {
        float roll_prop = (roll_wrapped - roll_limit_margin) / (float)(9000 - roll_limit_margin);
        // roll_prop = constrain_float(roll_prop, 0.01f, 1.0f); 
        // hal.console -> printf ("%f\n",roll_prop);
        demanded_pitch *= (1 - roll_prop);
    }

    if(underspeed){
        demanded_pitch = 0.0f;
    }//低速还没有舵效，会导致一直积分到很大值!!!!!!!!!!!!!!!

    // if(aspeed <= 12.0f){
    // // if (demanded_pitch < -radians(2.0f)) {
    // //     demanded_pitch = - radians(2.0f);
    // // } else if (demanded_pitch > radians(2.0f)) {
    // //     demanded_pitch = radians(2.0f);
    // // } 
    // }
    //以上俯仰限制


    // //20241125增量动态逆用 不稳定度大了暂时还不能航迹规划，舵机约束！
    const my &_my = AP::My();

    //期望角速度-角速度
    Vector3f crcpcy = multiplyMatrixVector(_my.last_zitai_NI, {demanded_roll,demanded_pitch ,0.0f}); 
    // float testx = crcpcy.x - rate_x;
    // float testy = crcpcy.y - rate_y + 0*(0 - radians(aoa));//加入迎角反馈2.0加10度预配平效果很好 + 0.0f*(0 - radians(aoa))
    float testz = crcpcy.z - rate_z;

    //调用自己改的PID
    float testflite_x = rate_pid_testx.update_all_my_x(crcpcy.x , rate_x , dt, aspeed);
    float testflite_y = rate_pid_testy.update_all_my_y(crcpcy.y , rate_y , dt, aspeed);

    //调用自己写的PID
    // //pitch计算P
    // float boost_pitch_p = 1.0f;
    // if(aspeed > 0.0f){
    //     boost_pitch_p =1.0f - 1.0f/20.0f*((abs(aspeed)));
    // }
    // boost_pitch_p = constrain_float(boost_pitch_p, 0.005f,1.0f);
    // float out_my_p_pitch = boost_pitch_p * testy;

    // //pitch计算D
    // error_my = testy;
    // float boost_pitch_d = 0.3f;
    // if(aspeed > 0.0f){
    //     boost_pitch_d  = 0.3 - 0.3f/20*((abs(aspeed)));
    // }
    // boost_pitch_d = constrain_float(boost_pitch_d, 0.05f,0.3f);
    // derivative = (error_my - previous_error) / dt;
    // float out_my_d_pitch = boost_pitch_d * derivative;
    // previous_error = error_my;


    // //roll计算P
    // float boost_roll_p = 1.0f;
    // if(aspeed > 0.0f){
    //     boost_roll_p =1.0f - 1.0f/20.0f*((abs(aspeed)));
    // }
    // boost_roll_p = constrain_float(boost_roll_p, 0.005f,1.0f);
    // float out_my_p_roll = boost_roll_p * testx / 2.0f;

    // //roll计算D
    // error_my_roll = testx;
    // float boost_roll_d = 0.3f;
    // if(aspeed > 0.0f){
    //     boost_roll_d =0.3f - 0.3f/20*((abs(aspeed)));
    // }
    // boost_roll_d = constrain_float(boost_roll_d, 0.05f,0.3f);
    // derivative_roll = (error_my_roll - previous_error_roll) / dt;
    // float out_my_d_roll = boost_roll_d * derivative_roll / 2.0f;
    // previous_error_roll = error_my_roll;


    // // // //滤波部分????
    // //实际角加速度滤波
    // float q = rate_y;
    // float derivativeq = (q - lastq) / dt;
    // lastq = q;
    // //角加速度的导数
    // float qdotdot = (derivativeq - lastderivativeq) / dt;
    // lastderivativeq = derivativeq;

    // KalmanFilter_my &_kf_my = AP::KalmanFilter_My();
    // // 模拟角速度测量数据（实际应用中应替换为真实数据）
    // // float measurement_my[3] = {0.1, 0.05, 0.0};
    // float measurement_my[3] = {q, derivativeq, qdotdot};
    // _kf_my.predict();
    // _kf_my.update(measurement_my);

    // hal.console -> printf("%f,%f\n",derivativeq,measurement_my[1]);
    // // //滤波部分

    if (underspeed) {
        testz = 0;
        // testx = 0;
        // testy = 0;
        // out_my_p_pitch = 0;
        // out_my_d_pitch = 0;
        // out_my_p_roll = 0;
        // out_my_d_roll = 0;

        // _derivativeq = 0;
        // _error = 0;
        // aoa = 0;
        testflite_x = 0;
        testflite_y = 0;
    }

    Vector3f dadedr = multiplyMatrixVector(_my.last_NI, { testflite_x ,testflite_y  ,testz}); 
    // hal.console -> printf("%f,%f\n",out_my_p_pitch + out_my_d_pitch,testflite_y); 
    // hal.console -> printf("%f,%f\n",out_my_p_roll + out_my_d_roll,testflite_x); 
    _lastout.x += dadedr.x;
    _lastout.y += dadedr.y;
    _lastout.z += dadedr.z * 0;

    _lastout.x = constrain_float(_lastout.x, -25.0f, 25.0f);
    _lastout.y = constrain_float(_lastout.y , -25.0f, 25.0f);
    _lastout.z = constrain_float(_lastout.z, -25.0f, 25.0f); 

    Vector3<float> constrained_out;
    constrained_out.x = map_range(_lastout.x, -25.0f, 25.0f, -4500, 4500);
    constrained_out.y = map_range(_lastout.y, -25.0f, 25.0f, -4500, 4500);
    constrained_out.z = map_range(_lastout.z, -25.0f, 25.0f, -4500, 4500);

    return constrained_out;
}






Vector3<float> Plane::stabilize_get_rpy_out(float pdot, float qdpt)
{
    const AP_AHRS &_ahrs = AP::ahrs();
    float aspeed;
    //低空速修正
    if (!_ahrs.airspeed_estimate(aspeed)) {
        aspeed = 0;
    }
    bool underspeed = aspeed <= 3.0f;
    // float rate_y = _ahrs.get_gyro().y;
    // float rate_x = _ahrs.get_gyro().x;
    float rate_z = _ahrs.get_gyro().z;

    // float demanded_yaw = radians((0 - ahrs.yaw_sensor)*0.01);

    // //20241125增量动态逆用
    const my &_my = AP::My();

    // Vector3f crcpcy = multiplyMatrixVector(_my.last_zitai_NI, {pdot,qdpt ,demanded_yaw}); 
    // float testr = crcpcy.x - rate_x;
    // float testp = crcpcy.y - rate_y;
    float testy = 0 - rate_z;

    if (underspeed) {
        // testr = 0;
        // testp = 0;
        testy = 0;
    }
    Vector3f dadedr = multiplyMatrixVector(_my.last_NI, {pdot,qdpt ,testy}); 


    _lastout.x += dadedr.x;
    _lastout.y += dadedr.y;
    _lastout.z += dadedr.z * 0;

    _lastout.x = constrain_float(_lastout.x, -15.0f, 15.0f);
    _lastout.y = constrain_float(_lastout.y, -15.0f, 15.0f);
    _lastout.z = constrain_float(_lastout.z, -15.0f, 15.0f); 

    Vector3<float> constrained_out;
    constrained_out.x = map_range(_lastout.x, -15.0f, 15.0f, -4500, 4500);
    constrained_out.y = map_range(_lastout.y, -15.0f, 15.0f, -4500, 4500);
    constrained_out.z = map_range(_lastout.z, -15.0f, 15.0f, -4500, 4500);


    return constrained_out;
}


Vector3<float> Plane::stabilize_get_rpy_out_2(float roll, float pitch)
{
    const AP_AHRS &_ahrs = AP::ahrs();
    float aspeed;
    //低空速修正
    if (!_ahrs.airspeed_estimate(aspeed)) {
        aspeed = 0;
    }
    bool underspeed = aspeed <= 3.0f;
    float rate_y = _ahrs.get_gyro().y;
    float rate_x = _ahrs.get_gyro().x;
    float rate_z = _ahrs.get_gyro().z;
    float demanded_roll =radians(roll*0.01)-ahrs.get_roll();   
    float demanded_pitch = radians(pitch*0.01)-ahrs.get_pitch();
    float demanded_yaw = radians((0 - ahrs.yaw_sensor)*0.01);

    // //20241125增量动态逆用
    const my &_my = AP::My();

    Vector3f crcpcy = multiplyMatrixVector(_my.last_zitai_NI, {demanded_roll,demanded_pitch ,demanded_yaw}); 
    float testr = crcpcy.x - rate_x;
    float testp = crcpcy.y - rate_y;
    float testy = 0 - rate_z;

    if (underspeed) {
        testr = 0;
        testp = 0;
        testy = 0;
    }
    Vector3f dadedr = multiplyMatrixVector(_my.last_NI, {testr,testp ,testy}); 


    _lastout.x += dadedr.x;
    _lastout.y += dadedr.y;
    _lastout.z += dadedr.z * 0;

    _lastout.x = constrain_float(_lastout.x, -15.0f, 15.0f);
    _lastout.y = constrain_float(_lastout.y, -15.0f, 15.0f);
    _lastout.z = constrain_float(_lastout.z, -15.0f, 15.0f); 

    Vector3<float> constrained_out;
    constrained_out.x = map_range(_lastout.x, -15.0f, 15.0f, -4500, 4500);
    constrained_out.y = map_range(_lastout.y, -15.0f, 15.0f, -4500, 4500);
    constrained_out.z = map_range(_lastout.z, -15.0f, 15.0f, -4500, 4500);


    return constrained_out;
}


// 滤波部分
float Plane::get_filt_T_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, 0.666);
}

// get_filt_E_alpha - get the error filter alpha
float Plane::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, 3);
}

// get_filt_D_alpha - get the derivative filter alpha
float Plane::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, 1);
}







//协调转弯（Coordination Turn） 所需的速率偏移量 rate_offset并根据飞行器的飞行姿态、空速和倾斜角进行调整。这通常用于飞行控制系统中的 转弯协调，确保飞行器在转弯时能保持协调状态，避免滑动或倾斜失控。
float Plane::_get_coordination_rate_offset_my(float &aspeed, bool &inverted) const
{
    float rate_offset;
    float bank_angle = AP::ahrs().get_roll();

    // limit bank angle between +- 80 deg if right way up
    if (fabsf(bank_angle) < radians(90))	{
        bank_angle = constrain_float(bank_angle,-radians(80),radians(80));
        inverted = false;
    } else {
        inverted = true;
        if (bank_angle > 0.0f) {
            bank_angle = constrain_float(bank_angle,radians(100),radians(180));
        } else {
            bank_angle = constrain_float(bank_angle,-radians(180),-radians(100));
        }
    }
    const AP_AHRS &_ahrs = AP::ahrs();
    if (!_ahrs.airspeed_estimate(aspeed)) {
        // If no airspeed available use average of min and max
        aspeed = 0.5f*(float(aparm.airspeed_min) + float(aparm.airspeed_max));
    }
    if (abs(_ahrs.pitch_sensor) > 7000) {
        // don't do turn coordination handling when at very high pitch angles
        rate_offset = 0;
    } else {
        rate_offset = cosf(_ahrs.get_pitch())*fabsf(ToDeg((GRAVITY_MSS / MAX((aspeed * _ahrs.get_EAS2TAS()), MAX(aparm.airspeed_min, 1))) * tanf(bank_angle) * sinf(bank_angle))) * 1.0f;
    }
    if (inverted) {
        rate_offset = -rate_offset;
    }
    return rate_offset;
}


float Plane::map_range(float value, float from_min, float from_max, float to_min, float to_max) {
    return to_min + (value - from_min) * (to_max - to_min) / (from_max - from_min);
}


Vector3f Plane::multiplyMatrixVector(const float matrix[3][3], const Vector3f& vec) {
    Vector3f result;
    result.x = matrix[0][0] * vec.x + matrix[0][1] * vec.y + matrix[0][2] * vec.z;
    result.y = matrix[1][0] * vec.x + matrix[1][1] * vec.y + matrix[1][2] * vec.z;
    result.z = matrix[2][0] * vec.x + matrix[2][1] * vec.y + matrix[2][2] * vec.z;
    return result;
}














/*
  main stabilization function for all 3 axes它是一个通用的姿态控制接口，负责在不同模式下根据控制模式的要求稳定飞机
 */
void Plane::stabilize()
{
    uint32_t now = AP_HAL::millis();
#if HAL_QUADPLANE_ENABLED
    if (quadplane.available()) {
        quadplane.transition->set_FW_roll_pitch(nav_pitch_cd, nav_roll_cd);
    }
#endif

    if (now - last_stabilize_ms > 2000) {
        // if we haven't run the rate controllers for 2 seconds then reset超过 2 秒没有运行，则重置控制器
        control_mode->reset_controllers();
    }
    last_stabilize_ms = now;

    if (control_mode == &mode_training ||
            control_mode == &mode_manual) {
        plane.control_mode->run();
#if AP_SCRIPTING_ENABLED//重点:脚本控制模式
    } else if (nav_scripting_active()) {
        // scripting is in control of roll and pitch rates and throttle
        const float speed_scaler = get_speed_scaler();
        const float aileron = rollController.get_rate_out(nav_scripting.roll_rate_dps, speed_scaler);
        const float elevator = pitchController.get_rate_out(nav_scripting.pitch_rate_dps, speed_scaler);
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, aileron);
        SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, elevator);
        float rudder = 0;
        if (yawController.rate_control_enabled()) {
            rudder = nav_scripting.rudder_offset_pct * 45;
            if (nav_scripting.run_yaw_rate_controller) {
                rudder += yawController.get_rate_out(nav_scripting.yaw_rate_dps, speed_scaler, false);
            } else {
                yawController.reset_I();
            }
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_steering, rudder);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.nav_scripting.throttle_pct);
#endif
    } else {
        plane.control_mode->run();
    }

    /*
      see if we should zero the attitude controller integrators. 
      在满足特定条件时重置姿态控制器的积分器。
      该逻辑主要用于在飞机静止或低速、低高度、无油门输入的情况下，防止积分器累积，从而避免在起飞前因积分器值过大导致控制问题
     */
    if (is_zero(get_throttle_input()) &&
        fabsf(relative_altitude) < 5.0f && 
        fabsf(barometer.get_climb_rate()) < 0.5f &&
        ahrs.groundspeed() < 3) {
        // we are low, with no climb rate, and zero throttle, and very
        // low ground speed. Zero the attitude controller
        // integrators. This prevents integrator buildup pre-takeoff.
        rollController.reset_I();
        pitchController.reset_I();
        yawController.reset_I();

        // if moving very slowly also zero the steering integrator
        if (ahrs.groundspeed() < 1) {
            steerController.reset_I();            
        }
    }
}


/*
 * Set the throttle output.
 * This is called by TECS-enabled flight modes, e.g. AUTO, GUIDED, etc.
*/
void Plane::calc_throttle()
{
    if (aparm.throttle_cruise <= 1) {
        // user has asked for zero throttle - this may be done by a
        // mission which wants to turn off the engine for a parachute
        // landing
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        return;
    }

    // Read the TECS throttle output and set it to the throttle channel.
    float commanded_throttle = TECS_controller.get_throttle_demand();
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, commanded_throttle);
}

/*****************************************
* Calculate desired roll/pitch/yaw angles (in medium freq loop)
*****************************************/

/*
  calculate yaw control for coordinated flight
 */
int16_t Plane::calc_nav_yaw_coordinated()
{
    const float speed_scaler = get_speed_scaler();
    bool disable_integrator = false;
    int16_t rudder_in = rudder_input();

    int16_t commanded_rudder;
    bool using_rate_controller = false;

    // Received an external msg that guides yaw in the last 3 seconds?
    if (control_mode->is_guided_mode() &&
            plane.guided_state.last_forced_rpy_ms.z > 0 &&
            millis() - plane.guided_state.last_forced_rpy_ms.z < 3000) {
        commanded_rudder = plane.guided_state.forced_rpy_cd.z;
    } else if (autotuning && g.acro_yaw_rate > 0 && yawController.rate_control_enabled()) {
        // user is doing an AUTOTUNE with yaw rate control
        const float rudd_expo = rudder_in_expo(true);
        const float yaw_rate = (rudd_expo/SERVO_MAX) * g.acro_yaw_rate;
        // add in the coordinated turn yaw rate to make it easier to fly while tuning the yaw rate controller
        const float coordination_yaw_rate = degrees(GRAVITY_MSS * tanf(radians(nav_roll_cd*0.01f))/MAX(aparm.airspeed_min,smoothed_airspeed));
        commanded_rudder = yawController.get_rate_out(yaw_rate+coordination_yaw_rate,  speed_scaler, false);
        using_rate_controller = true;
    } else {
        if (control_mode == &mode_stabilize && rudder_in != 0) {
            disable_integrator = true;
        }

        commanded_rudder = yawController.get_servo_out(speed_scaler, disable_integrator);

        // add in rudder mixing from roll
        commanded_rudder += SRV_Channels::get_output_scaled(SRV_Channel::k_aileron) * g.kff_rudder_mix;
        commanded_rudder += rudder_in;
    }

    if (!using_rate_controller) {
        /*
          When not running the yaw rate controller, we need to reset the rate
        */
        yawController.reset_rate_PID();
    }

    return constrain_int16(commanded_rudder, -4500, 4500);
}

/*
  calculate yaw control for ground steering with specific course
 */
int16_t Plane::calc_nav_yaw_course(void)
{
    // holding a specific navigation course on the ground. Used in
    // auto-takeoff and landing
    int32_t bearing_error_cd = nav_controller->bearing_error_cd();
    int16_t steering = steerController.get_steering_out_angle_error(bearing_error_cd);
    if (stick_mixing_enabled()) {
        steering = channel_rudder->stick_mixing(steering);
    }
    return constrain_int16(steering, -4500, 4500);
}

/*
  calculate yaw control for ground steering
 */
int16_t Plane::calc_nav_yaw_ground(void)
{
    if (gps.ground_speed() < 1 && 
        is_zero(get_throttle_input()) &&
        flight_stage != AP_FixedWing::FlightStage::TAKEOFF &&
        flight_stage != AP_FixedWing::FlightStage::ABORT_LANDING) {
        // manual rudder control while still
        steer_state.locked_course = false;
        steer_state.locked_course_err = 0;
        return rudder_input();
    }

    // if we haven't been steering for 1s then clear locked course
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - steer_state.last_steer_ms > 1000) {
        steer_state.locked_course = false;
    }
    steer_state.last_steer_ms = now_ms;

    float steer_rate = (rudder_input()/4500.0f) * g.ground_steer_dps;
    if (flight_stage == AP_FixedWing::FlightStage::TAKEOFF ||
        flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING) {
        steer_rate = 0;
    }
    if (!is_zero(steer_rate)) {
        // pilot is giving rudder input
        steer_state.locked_course = false;        
    } else if (!steer_state.locked_course) {
        // pilot has released the rudder stick or we are still - lock the course
        steer_state.locked_course = true;
        if (flight_stage != AP_FixedWing::FlightStage::TAKEOFF &&
            flight_stage != AP_FixedWing::FlightStage::ABORT_LANDING) {
            steer_state.locked_course_err = 0;
        }
    }

    int16_t steering;
    if (!steer_state.locked_course) {
        // use a rate controller at the pilot specified rate
        steering = steerController.get_steering_out_rate(steer_rate);
    } else {
        // use a error controller on the summed error
        int32_t yaw_error_cd = -ToDeg(steer_state.locked_course_err)*100;
        steering = steerController.get_steering_out_angle_error(yaw_error_cd);
    }
    return constrain_int16(steering, -4500, 4500);
}


/*
  calculate a new nav_pitch_cd from the speed height controller
 */
void Plane::calc_nav_pitch()
{
    int32_t commanded_pitch = TECS_controller.get_pitch_demand();
    nav_pitch_cd = constrain_int32(commanded_pitch, pitch_limit_min*100, aparm.pitch_limit_max.get()*100);
}


/*
  calculate a new nav_roll_cd from the navigation controller
 */
void Plane::calc_nav_roll()
{
    int32_t commanded_roll = nav_controller->nav_roll_cd();
    nav_roll_cd = constrain_int32(commanded_roll, -roll_limit_cd, roll_limit_cd);
    update_load_factor();
}

/*
  adjust nav_pitch_cd for STAB_PITCH_DOWN_CD. This is used to make
  keeping up good airspeed in FBWA mode easier, as the plane will
  automatically pitch down a little when at low throttle. It makes
  FBWA landings without stalling much easier.
 */
void Plane::adjust_nav_pitch_throttle(void)
{
    int8_t throttle = throttle_percentage();
    if (throttle >= 0 && throttle < aparm.throttle_cruise && flight_stage != AP_FixedWing::FlightStage::VTOL) {
        float p = (aparm.throttle_cruise - throttle) / (float)aparm.throttle_cruise;
        nav_pitch_cd -= g.stab_pitch_down * 100.0f * p;
    }
}


/*
  calculate a new aerodynamic_load_factor and limit nav_roll_cd to
  ensure that the load factor does not take us below the sustainable
  airspeed
 */
void Plane::update_load_factor(void)
{
    float demanded_roll = fabsf(nav_roll_cd*0.01f);
    if (demanded_roll > 85) {
        // limit to 85 degrees to prevent numerical errors
        demanded_roll = 85;
    }
    aerodynamic_load_factor = 1.0f / safe_sqrt(cosf(radians(demanded_roll)));

#if HAL_QUADPLANE_ENABLED
    if (quadplane.available() && quadplane.transition->set_FW_roll_limit(roll_limit_cd)) {
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
        return;
    }
#endif

    if (!aparm.stall_prevention) {
        // stall prevention is disabled
        return;
    }
    if (fly_inverted()) {
        // no roll limits when inverted
        return;
    }
#if HAL_QUADPLANE_ENABLED
    if (quadplane.tailsitter.active()) {
        // no limits while hovering
        return;
    }
#endif

    float max_load_factor = smoothed_airspeed / MAX(aparm.airspeed_min, 1);
    if (max_load_factor <= 1) {
        // our airspeed is below the minimum airspeed. Limit roll to
        // 25 degrees
        nav_roll_cd = constrain_int32(nav_roll_cd, -2500, 2500);
        roll_limit_cd = MIN(roll_limit_cd, 2500);
    } else if (max_load_factor < aerodynamic_load_factor) {
        // the demanded nav_roll would take us past the aerodynamic
        // load limit. Limit our roll to a bank angle that will keep
        // the load within what the airframe can handle. We always
        // allow at least 25 degrees of roll however, to ensure the
        // aircraft can be manoeuvred with a bad airspeed estimate. At
        // 25 degrees the load factor is 1.1 (10%)
        int32_t roll_limit = degrees(acosf(sq(1.0f / max_load_factor)))*100;
        if (roll_limit < 2500) {
            roll_limit = 2500;
        }
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit, roll_limit);
        roll_limit_cd = MIN(roll_limit_cd, roll_limit);
    }    
}

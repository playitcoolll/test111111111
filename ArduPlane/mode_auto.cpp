#include "mode.h"
#include "Plane.h"

bool ModeAuto::_enter()
{
#if HAL_QUADPLANE_ENABLED
    // check if we should refuse auto mode due to a missing takeoff in
    // guided_wait_takeoff state
    if (plane.previous_mode == &plane.mode_guided &&
        quadplane.guided_wait_takeoff_on_mode_enter) {
        if (!plane.mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_ERROR,"Takeoff waypoint required");
            quadplane.guided_wait_takeoff = true;
            return false;
        }
    }
    
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        plane.auto_state.vtol_mode = true;
    } else {
        plane.auto_state.vtol_mode = false;
    }
#else
    plane.auto_state.vtol_mode = false;
#endif
    //上面的不用看
    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();

    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }
    /*
    这部分检查看门狗计时器（watchdog）是否已经启用。如果启用了，并且有之前的航点任务存储在 persistent_data.waypoint_num 中，
    那么会从这个航点编号恢复任务执行，并通过 GCS 发送恢复任务的信息。
    这一部分的作用是如果因为某些故障（如电源问题或其他原因）中断了任务，能够从中断处恢复继续执行。
    */

#if HAL_SOARING_ENABLED
    plane.g2.soaring_controller.init_cruising();
#endif

    return true;
}

void ModeAuto::_exit()//飞行模式切换\任务完成\传感器或系统故障等等情况，飞行器将自动退出自动模式
{
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        plane.mission.stop();//这部分代码首先检查当前任务的状态，plane.mission.state() 返回的是任务的运行状态。如果任务正在运行（即任务状态是 MISSION_RUNNING），那么执行 plane.mission.stop() 停止任务的运行。
    
        bool restart = plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND;//检查当前任务是否为着陆命令（MAV_CMD_NAV_LAND）。如果当前任务是着陆任务，则将 restart 设置为 true
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id)) {
            restart = false;
        }
#endif
        if (restart) {
            plane.landing.restart_landing_sequence();
        }//这意味着在退出自动模式时，如果飞行器正在执行固定翼的着陆任务，那么会重启整个着陆过程，确保着陆操作正确执行
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
}

void ModeAuto::update()
{
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in auto without a running mission");
        return;
    }

    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto()) {
        plane.quadplane.control_auto();
        return;
    }
#endif

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (pullup.in_pullup()) {
        return;
    }
#endif

    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING)) {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // allow landing to restrict the roll limits
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit*100UL);

        if (plane.landing.is_throttle_suppressed()) {
            // if landing is considered complete throttle is never allowed, regardless of landing type
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        } else {
            plane.calc_throttle();
        }
#if AP_SCRIPTING_ENABLED
    } else if (nav_cmd_id == MAV_CMD_NAV_SCRIPT_TIME) {
        // NAV_SCRIPTING has a desired roll and pitch rate and desired throttle
        plane.nav_roll_cd = ahrs.roll_sensor;
        plane.nav_pitch_cd = ahrs.pitch_sensor;
#endif
    } else {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

void ModeAuto::navigate()
{
    if (AP::ahrs().home_is_set()) {
        plane.mission.update();
    }
}


bool ModeAuto::does_auto_navigation() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

bool ModeAuto::does_auto_throttle() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

// returns true if the vehicle can be armed in this mode
bool ModeAuto::_pre_arm_checks(size_t buflen, char *buffer) const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.enabled()) {
        if (plane.quadplane.option_is_set(QuadPlane::OPTION::ONLY_ARM_IN_QMODE_OR_AUTO) &&
                !plane.quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)) {
            hal.util->snprintf(buffer, buflen, "not in VTOL takeoff");
            return false;
        }
        if (!plane.mission.starts_with_takeoff_cmd()) {
            hal.util->snprintf(buffer, buflen, "missing takeoff waypoint");
            return false;
        }
    }
#endif
    // Note that this bypasses the base class checks
    return true;
}

bool ModeAuto::is_landing() const
{
    return (plane.flight_stage == AP_FixedWing::FlightStage::LAND);
}

void ModeAuto::run()
{
#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (pullup.in_pullup()) {
        pullup.stabilize_pullup();
        return;
    }
#endif
    
    if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_ALTITUDE_WAIT) {

        wiggle_servos();

        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0);

        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttle);
        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttleLeft);
        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttleRight);

        // Relax attitude control
        reset_controllers();

    } else {
        // Normal flight, run base class
        Mode::run();

    }
}

/*
run 函数：通常是主循环中的核心函数，用于控制每个时间周期中执行的操作。run 函数负责调用整个控制系统的各个子模块，确保所有的飞行控制、导航和传感器处理都按照预定的频率进行。
update 函数：主要负责更新某个模块的状态或数据，例如更新传感器数据、计算姿态角、更新导航参数等。update 函数通常在某些特定模块或对象内被调用，以便更新该模块的状态，供 run 或其他主要控制函数使用
*/
#pragma once

#include <AP_Logger/LogStructure.h>
#include <AP_Param/AP_Param.h>
#include <AP_Vehicle/AP_FixedWing.h>
#include <Filter/SlewLimiter.h>

#include <Filter/ModeFilter.h>

class AP_AutoTune
{
public:
    struct ATGains {
        AP_Float tau;//时间常数，可能影响系统响应
        AP_Int16 rmax_pos;//正负最大限制，可能与伺服控制或激励输出相关。
        AP_Int16 rmax_neg;
        float FF, P, I, D, IMAX;//前馈增益（FF）、比例增益（P）、积分增益（I）、微分增益（D）、积分最大值（IMAX）
        float flt_T, flt_E, flt_D;//滤波器相关的参数
    };

    enum ATType {
        AUTOTUNE_ROLL  = 0,
        AUTOTUNE_PITCH = 1,
        AUTOTUNE_YAW = 2,
    };

    enum Options {
        DISABLE_FLTD_UPDATE = 0,//禁用滤波器更新
        DISABLE_FLTT_UPDATE = 1,//禁用另一个滤波器更新
    };

    struct PACKED log_ATRP {
        LOG_PACKET_HEADER;
        uint64_t time_us;//日志时间戳
        uint8_t type;//自动调谐类型（如滚转、俯仰、偏航）
        uint8_t state;//当前状态
        float actuator;//控制器的执行器输出
        float P_slew;//比例和微分增益的变化
        float D_slew;
        float FF_single;//前馈、比例、积分和微分增益的具体值
        float FF;
        float P;
        float I;
        float D;
        uint8_t action;//当前动作（如调整P、D增益）
        float rmax;//最大控制量和时间常数
        float tau;
    };

    // constructor初始化调谐参数、PID控制器对象和其他相关参数
    AP_AutoTune(ATGains &_gains, ATType type, const AP_FixedWing &parms, class AC_PID &rpid);

    // called when autotune mode is entered启动自动调谐模式
    void start(void);

    // called to stop autotune and restore gains when user leaves停止自动调谐并恢复原始增益
    // autotune
    void stop(void);

    // update called whenever autotune mode is active. This is
    // called at the main loop rate每次主循环时更新自动调谐的状态，并根据误差调整PID增益
    void update(struct AP_PIDInfo &pid_info, float scaler, float angle_err_deg);

    // are we running?
    bool running;

private:
    // the current gains
    ATGains &current;
    class AC_PID &rpid;

    // what type of autotune is this
    ATType type;

    const AP_FixedWing &aparm;

    // values to restore if we leave autotune mode
    ATGains restore;
    ATGains last_save;

    // last logging time
    uint32_t last_log_ms;

    // the demanded/achieved state
    enum class ATState {IDLE,
                        DEMAND_POS,
                        DEMAND_NEG
                       };
    ATState state;

    // the demanded/achieved state
    enum class Action {NONE,
                       LOW_RATE,
                       SHORT,
                       RAISE_PD,
                       LOWER_PD,
                       IDLE_LOWER_PD,
                       RAISE_D,
                       RAISE_P,
                       LOWER_D,
                       LOWER_P
                      };
    Action action;

    // when we entered the current state
    uint32_t state_enter_ms;

    void check_state_exit(uint32_t state_time_ms);
    void save_gains(void);

    void save_float_if_changed(AP_Float &v, float value);
    void save_int16_if_changed(AP_Int16 &v, int16_t value);
    void state_change(ATState newstate);
    const char *axis_string(void) const;

    // get gains with PID components
    ATGains get_gains(void);
    void restore_gains(void);

    // update rmax and tau towards target
    void update_rmax();

    bool has_option(Options option) {
        return (aparm.autotune_options.get() & uint32_t(1<<uint32_t(option))) != 0;
    }

    // 5 point mode filter for FF estimate
    ModeFilterFloat_Size5 ff_filter;

    LowPassFilterConstDtFloat actuator_filter;
    LowPassFilterConstDtFloat rate_filter;
    LowPassFilterConstDtFloat target_filter;

    // separate slew limiters for P and D
    float slew_limit_max, slew_limit_tau;
    SlewLimiter slew_limiter_P{slew_limit_max, slew_limit_tau};
    SlewLimiter slew_limiter_D{slew_limit_max, slew_limit_tau};

    float max_actuator;
    float min_actuator;
    float max_rate;
    float min_rate;
    float max_target;
    float min_target;
    float max_P;
    float max_D;
    float min_Dmod;
    float max_Dmod;
    float max_SRate_P;
    float max_SRate_D;
    float FF_single;
    uint16_t ff_count;
    float dt;
    float D_limit;
    float P_limit;
    uint32_t D_set_ms;
    uint32_t P_set_ms;
    uint8_t done_count;
};

#include "screw_axis.h"

#include <stdint.h>

#include "BoardConfig.h"
#include "MotorControl.h"
#include "TuneConfig.h"
#include "foc_math.h"
#include "motor_control_state.h"
#include "motor_controller.hpp"

#include "clamp.hpp"

namespace
{

using cms32::motor::g_motor;
using cms32::support::clamp;

constexpr uint8_t kMotorModeSpeed = 2U;
constexpr uint8_t kMotorModePosition = MC_MODE_POSITION;
constexpr int16_t kElecZeroTrim = 0;

struct HomeConfig
{

    static constexpr int16_t fast_retract_rpm = -2000;
    static constexpr int16_t slow_retract_rpm = -50;
    static constexpr int16_t extend_rpm = 2000;
    static constexpr int16_t backoff_rpm = 300;
    static constexpr int16_t iq_limit = 24;
    static constexpr uint16_t timeout_ms = 12000U;
    static constexpr uint8_t coarse_retract_hits = 2U;

    // 通过“速度接近 0 且 q 轴命令接近限幅”判断机械端点。
    static constexpr int16_t stall_speed_rpm = 8;
    static constexpr int16_t stall_iq_margin = 2;
    static constexpr uint32_t fast_stall_hold_ms = 100U;
    static constexpr uint32_t slow_stall_hold_ms = 250U;
    static constexpr uint32_t stall_clear_hold_ms = 50U;
    static constexpr uint32_t fast_backoff_ms = 300U;
    static constexpr uint32_t final_backoff_ms = 300U;
};

struct PositionConfig
{
    // 0.50 mm/rev，用 mm_x100 表示就是 50。
    static constexpr int32_t lead_mm_x100 = MOT_SCREW_LEAD_MM_X100;
    static constexpr int32_t nominal_travel_mm_x100 = MOT_SCREW_NOMINAL_TRAVEL_MM_X100;
    static constexpr int32_t counts_per_rev = MOT_POS_COUNTS_PER_REV;
    static constexpr int16_t speed_limit_rpm = CTRL_POS_SPEED_LIMIT_RPM;
    static constexpr int16_t iq_limit = CTRL_POS_IQ_LIMIT;
    static constexpr int16_t kp = CTRL_POS_KP;
    static constexpr uint8_t err_shift = CTRL_POS_ERR_SHIFT;
    static constexpr int32_t deadband_counts = CTRL_POS_DEADBAND_COUNTS;
    static constexpr int32_t sine_center_mm_x100 = 500;
    static constexpr int32_t sine_amplitude_mm_x100 = 200;
    static constexpr uint16_t sine_period_ms = 2000U;
    static constexpr int32_t test_center_mm_x100 = 500;
    static constexpr int32_t test_amplitude_mm_x100 = 100;
    static constexpr uint16_t test_frequency_mhz = 500U;
    static constexpr uint16_t test_sweep_start_frequency_mhz = 100U;
    static constexpr uint16_t test_sweep_end_frequency_mhz = 5000U;
    static constexpr uint32_t test_sweep_duration_ms = 30000U;
};

enum class HomeState : uint8_t
{
    Idle = ScrewHomeState_Idle,
    SeekOuter = ScrewHomeState_SeekOuter,
    FastRetract = ScrewHomeState_FastRetract,
    FastBackoff = ScrewHomeState_FastBackoff,
    SlowRetract = ScrewHomeState_SlowRetract,
    FinalBackoff = ScrewHomeState_FinalBackoff,
    Done = ScrewHomeState_Done,
    Fault = ScrewHomeState_Fault,
};

enum class AxisState : uint8_t
{
    Idle = ScrewAxisState_Idle,
    Homing = ScrewAxisState_Homing,
    Ready = ScrewAxisState_Ready,
    Fault = ScrewAxisState_Fault,
};

enum class TestMode : uint8_t
{
    Off = ScrewAxisTestMode_Off,
    Sine = ScrewAxisTestMode_Sine,
    Step = ScrewAxisTestMode_Step,
    Sweep = ScrewAxisTestMode_Sweep,
};

struct HomeRuntime
{
    HomeState state{HomeState::Idle};
    // phase_start_ms 是当前状态阶段的起点，用来做超时和固定退离时间判断。
    uint32_t phase_start_ms{0U};
    // stall_start_ms / stall_clear_start_ms 用来过滤低速估算抖动，避免单点误判。
    uint32_t stall_start_ms{0U};
    uint32_t stall_clear_start_ms{0U};
    // 快速收回撞到内端的次数。当前先做两次外伸/内收粗测试，再低速精找零。
    uint8_t fast_retract_hits{0U};
    // 以下参数在 start_home() 里锁存，避免回零运行中 Ozone 改参数导致状态跳变。
    int16_t extend_speed_rpm{HomeConfig::extend_rpm};
    int16_t fast_speed_rpm{HomeConfig::fast_retract_rpm};
    int16_t slow_speed_rpm{HomeConfig::slow_retract_rpm};
    int16_t backoff_speed_rpm{HomeConfig::backoff_rpm};
    int16_t iq_limit{HomeConfig::iq_limit};
    uint16_t timeout_ms{HomeConfig::timeout_ms};
    bool homed{false};
    bool outer_seen{false};
    bool fault_seen{false};
    uint8_t fault_reason{ScrewAxisFault_None};
    // 这两个位置都是 g_motor.encoder.pos 的 RAM 快照，断电后不会保留。
    int32_t zero_encoder_pos{0};
    int32_t outer_encoder_pos{0};
};

struct PositionRuntime
{
    bool active{false};
    bool sine_active{false};
    bool test_active{false};
    uint32_t sine_start_ms{0U};
    uint32_t test_start_ms{0U};
    int32_t target_mm_x100{0};
    int32_t sine_target_mm_x100{0};
    int32_t test_target_mm_x100{0};
    int32_t target_counts{0};
    int32_t absolute_position_ref{0};
    int16_t speed_limit_rpm{PositionConfig::speed_limit_rpm};
    int16_t iq_limit{PositionConfig::iq_limit};
    int16_t kp{PositionConfig::kp};
    uint8_t err_shift{PositionConfig::err_shift};
    int32_t deadband_counts{PositionConfig::deadband_counts};
    uint16_t sine_phase{0U};
    uint16_t test_phase{0U};
    uint16_t test_frequency_mhz{0U};
    TestMode test_mode{TestMode::Off};
};

// s_adc_samples 只在 ADC 有效控制采样后递增，所以 app_millis() 跟控制采样同步。
volatile uint32_t s_adc_samples;
HomeRuntime s_home;
PositionRuntime s_position;
AxisState s_axis_state{AxisState::Idle};
int16_t s_axis_speed_cmd_rpm{0};

constexpr int16_t abs_i16(int16_t value) noexcept
{
    if (value >= 0)
    {
        return value;
    }
    if (value == INT16_MIN)
    {
        return INT16_MAX;
    }
    return static_cast<int16_t>(-value);
}

uint32_t app_millis() noexcept
{
    return s_adc_samples / (PWM_FREQ_HZ / 1000U);
}

// 收回速度无论用户写正写负，最终都变成负 rpm。
int16_t normalize_retract_speed(int16_t value, int16_t fallback) noexcept
{
    int16_t speed = abs_i16(value);
    if (speed == 0)
    {
        speed = abs_i16(fallback);
    }
    return static_cast<int16_t>(-speed);
}

// 伸出/退离速度无论用户写正写负，最终都变成正 rpm。
int16_t normalize_extend_speed(int16_t value, int16_t fallback) noexcept
{
    int16_t speed = abs_i16(value);
    if (speed == 0)
    {
        speed = fallback;
    }
    return speed;
}

int32_t travel_counts() noexcept
{
    // 外端记录后就发布实测行程；它只是观察值，不参与当前简化找零判定。
    if (!s_home.outer_seen)
    {
        return 0;
    }

    return s_home.outer_encoder_pos - s_home.zero_encoder_pos;
}

int32_t current_position_counts() noexcept
{
    // 未回零前没有可信机械零点，因此不发布相对位置。
    if (!s_home.homed)
    {
        return 0;
    }

    return g_motor.encoder.pos - s_home.zero_encoder_pos;
}

int32_t counts_to_mm_x100(int32_t counts) noexcept
{
    const int64_t scaled = static_cast<int64_t>(counts) * PositionConfig::lead_mm_x100;
    return static_cast<int32_t>(scaled / PositionConfig::counts_per_rev);
}

int32_t mm_x100_to_counts(int32_t mm_x100) noexcept
{
    const int64_t scaled =
        static_cast<int64_t>(mm_x100) * PositionConfig::counts_per_rev;
    return static_cast<int32_t>(scaled / PositionConfig::lead_mm_x100);
}

int32_t travel_mm_x100() noexcept
{
    return counts_to_mm_x100(travel_counts());
}

bool home_complete() noexcept
{
    return s_home.homed && (s_home.state == HomeState::Done);
}

void publish_status() noexcept
{
    /*
     * watch 只做调试发布，不反过来参与控制。
     * 控制真正使用的是 s_home 和 g_motor.*，这样 Ozone
     * 看到的是状态快照而不是第二套状态机。
     */
    const bool busy = (s_home.state != HomeState::Idle) &&
                      (s_home.state != HomeState::Done) &&
                      (s_home.state != HomeState::Fault);

    const int32_t pos_counts = current_position_counts();
    const int32_t actual_mm_x100 = counts_to_mm_x100(pos_counts);
    const int32_t travel_count_snapshot = travel_counts();
    const int32_t travel_mm_snapshot = counts_to_mm_x100(travel_count_snapshot);
    const int32_t error_mm_x100 = s_position.target_mm_x100 - actual_mm_x100;
    const uint8_t position_active = s_position.active ? 1U : 0U;
    const uint8_t sine_active = s_position.sine_active ? 1U : 0U;

    g_screw_axis_watch.state = static_cast<uint8_t>(s_axis_state);
    g_screw_axis_watch.homed = home_complete() ? 1U : 0U;
    g_screw_axis_watch.fault_seen = s_home.fault_seen ? 1U : 0U;
    g_screw_axis_watch.fault_reason = s_home.fault_reason;
    g_screw_axis_watch.moving = busy ? 1U : 0U;
    g_screw_axis_watch.zero_encoder_pos = s_home.zero_encoder_pos;
    g_screw_axis_watch.outer_encoder_pos = s_home.outer_encoder_pos;
    g_screw_axis_watch.travel_counts = travel_count_snapshot;
    g_screw_axis_watch.pos_counts = pos_counts;
    g_screw_axis_watch.lead_mm_x100 = PositionConfig::lead_mm_x100;
    g_screw_axis_watch.counts_per_rev = PositionConfig::counts_per_rev;
    g_screw_axis_watch.nominal_travel_mm_x100 = PositionConfig::nominal_travel_mm_x100;
    g_screw_axis_watch.travel_mm_x100 = travel_mm_snapshot;
    g_screw_axis_watch.pos_mm_x100 = actual_mm_x100;
    g_screw_axis_watch.target_mm_x100 = s_position.target_mm_x100;
    g_screw_axis_watch.error_mm_x100 = error_mm_x100;
    g_screw_axis_watch.sine_target_mm_x100 = s_position.sine_target_mm_x100;
    g_screw_axis_watch.sine_phase = s_position.sine_phase;
    g_screw_axis_watch.sine_active = sine_active;
    g_screw_axis_watch.target_counts = s_position.target_counts;
    g_screw_axis_watch.absolute_position_ref = s_position.absolute_position_ref;
    g_screw_axis_watch.position_active = position_active;
    g_screw_axis_watch.speed_cmd_rpm = s_axis_speed_cmd_rpm;
    g_screw_axis_watch.home_state = static_cast<uint8_t>(s_home.state);
    g_screw_axis_watch.test_mode = static_cast<uint8_t>(s_position.test_mode);
    g_screw_axis_watch.test_active = s_position.test_active ? 1U : 0U;
    g_screw_axis_watch.test_target_mm_x100 = s_position.test_target_mm_x100;
    g_screw_axis_watch.test_phase = s_position.test_phase;
    g_screw_axis_watch.test_frequency_mhz = s_position.test_frequency_mhz;
}

void command_motor_speed(int16_t speed_rpm, int16_t iq_limit) noexcept
{
    /*
     * ScrewAxis 不直接控制电流环或 PWM，只把机械回零动作转换成 MotorControl
     * 速度模式命令。 MotorControl_ApplyCommand() 会在主循环里把 g_mc_cmd
     * 锁存到控制内部。
     */
    g_mc_cmd.enable = 1U;
    g_mc_cmd.control_mode = kMotorModeSpeed;
    g_mc_cmd.id_ref = 0;
    g_mc_cmd.iq_ref = 0;
    g_mc_cmd.speed_ref = 0;
    g_mc_cmd.speed_ref_rpm = speed_rpm;
    g_mc_cmd.iq_limit = iq_limit;
    g_mc_cmd.elec_zero_trim = kElecZeroTrim;
    s_axis_speed_cmd_rpm = speed_rpm;
}

void command_home_speed(int16_t speed_rpm) noexcept
{
    command_motor_speed(speed_rpm, s_home.iq_limit);
}

void stop_motor(uint8_t keep_enabled) noexcept
{
    g_mc_cmd.enable = keep_enabled;
    g_mc_cmd.control_mode = kMotorModeSpeed;
    g_mc_cmd.id_ref = 0;
    g_mc_cmd.iq_ref = 0;
    g_mc_cmd.speed_ref = 0;
    g_mc_cmd.speed_ref_rpm = 0;
    g_mc_cmd.iq_limit = 0;
    s_axis_speed_cmd_rpm = 0;
}

int16_t normalize_position_speed_limit(int16_t value) noexcept
{
    int16_t speed = abs_i16(value);
    if (speed == 0)
    {
        speed = PositionConfig::speed_limit_rpm;
    }
    return speed;
}

int16_t normalize_position_iq_limit(int16_t value) noexcept
{
    int16_t limit = abs_i16(value);
    if (limit == 0)
    {
        limit = PositionConfig::iq_limit;
    }
    return limit;
}

int16_t normalize_position_kp(int16_t value) noexcept
{
    int16_t kp = abs_i16(value);
    if (kp == 0)
    {
        kp = PositionConfig::kp;
    }
    return kp;
}

uint8_t normalize_position_err_shift(uint8_t value) noexcept
{
    return (value == 0U) ? PositionConfig::err_shift : clamp<uint8_t>(value, 1U, 30U);
}

int32_t normalize_position_deadband_counts(int32_t value) noexcept
{
    const int32_t deadband = (value < 0) ? -value : value;
    if (deadband == 0)
    {
        return PositionConfig::deadband_counts;
    }
    return clamp<int32_t>(deadband, 0, PositionConfig::counts_per_rev);
}

bool position_enabled() noexcept
{
    return g_screw_axis_cmd.position_enable != 0U;
}

bool sine_enabled() noexcept
{
    return g_screw_axis_cmd.sine_enable != 0U;
}

int32_t normalize_sine_amplitude(int32_t value) noexcept
{
    return (value < 0) ? -value : value;
}

uint16_t sine_period_ms() noexcept
{
    const uint16_t value = g_screw_axis_cmd.sine_period_ms;
    return (value == 0U) ? PositionConfig::sine_period_ms : value;
}

int32_t sine_center_mm_x100() noexcept
{
    const int32_t value = g_screw_axis_cmd.sine_center_mm_x100;
    return (value == 0) ? PositionConfig::sine_center_mm_x100 : value;
}

int32_t sine_amplitude_mm_x100() noexcept
{
    const int32_t value = g_screw_axis_cmd.sine_amplitude_mm_x100;
    const int32_t amplitude = normalize_sine_amplitude(value);
    return (amplitude == 0) ? PositionConfig::sine_amplitude_mm_x100 : amplitude;
}

TestMode requested_test_mode() noexcept
{
    if (g_screw_axis_cmd.test_enable != 0U)
    {
        switch (static_cast<TestMode>(g_screw_axis_cmd.test_mode))
        {
        case TestMode::Sine:
        case TestMode::Step:
        case TestMode::Sweep:
            return static_cast<TestMode>(g_screw_axis_cmd.test_mode);

        case TestMode::Off:
        default:
            return TestMode::Off;
        }
    }

    // 兼容已有 Ozone 脚本：旧 sine_enable 等同于新 Sine 测试。
    return sine_enabled() ? TestMode::Sine : TestMode::Off;
}

bool legacy_sine_requested(TestMode mode) noexcept
{
    return (mode == TestMode::Sine) && (g_screw_axis_cmd.test_enable == 0U);
}

int32_t abs_i32(int32_t value) noexcept
{
    if (value >= 0)
    {
        return value;
    }
    return (value == INT32_MIN) ? INT32_MAX : -value;
}

int32_t test_center_mm_x100(TestMode mode) noexcept
{
    return legacy_sine_requested(mode) ? sine_center_mm_x100()
                                       : g_screw_axis_cmd.test_center_mm_x100;
}

int32_t test_amplitude_mm_x100(TestMode mode) noexcept
{
    return legacy_sine_requested(mode)
               ? sine_amplitude_mm_x100()
               : abs_i32(g_screw_axis_cmd.test_amplitude_mm_x100);
}

uint16_t fixed_test_frequency_mhz(TestMode mode) noexcept
{
    if (legacy_sine_requested(mode))
    {
        return static_cast<uint16_t>(1000000UL / sine_period_ms());
    }

    const uint16_t frequency = g_screw_axis_cmd.test_frequency_mhz;
    return (frequency == 0U) ? PositionConfig::test_frequency_mhz : frequency;
}

uint16_t sweep_start_frequency_mhz() noexcept
{
    const uint16_t frequency = g_screw_axis_cmd.test_sweep_start_frequency_mhz;
    return (frequency == 0U) ? PositionConfig::test_sweep_start_frequency_mhz
                             : frequency;
}

uint16_t sweep_end_frequency_mhz() noexcept
{
    const uint16_t frequency = g_screw_axis_cmd.test_sweep_end_frequency_mhz;
    return (frequency == 0U) ? PositionConfig::test_sweep_end_frequency_mhz : frequency;
}

uint32_t sweep_duration_ms() noexcept
{
    const uint32_t duration = g_screw_axis_cmd.test_sweep_duration_ms;
    return (duration == 0U) ? PositionConfig::test_sweep_duration_ms : duration;
}

uint16_t phase_from_millihz_ms(uint64_t frequency_time) noexcept
{
    constexpr uint64_t kMillihzMsPerCycle = UINT64_C(1000000);
    const uint64_t phase = (frequency_time * UINT64_C(65536)) / kMillihzMsPerCycle;
    return static_cast<uint16_t>(phase);
}

int32_t update_test_target_mm_x100(TestMode mode, uint32_t now_ms) noexcept
{
    if ((!s_position.test_active) || (s_position.test_mode != mode))
    {
        s_position.test_active = true;
        s_position.test_mode = mode;
        s_position.test_start_ms = now_ms;
    }

    const uint32_t elapsed_ms = now_ms - s_position.test_start_ms;
    uint16_t phase = 0U;
    uint16_t frequency_mhz = 0U;

    if (mode == TestMode::Sweep)
    {
        const uint32_t duration_ms = sweep_duration_ms();
        const uint32_t sweep_ms = (elapsed_ms < duration_ms) ? elapsed_ms : duration_ms;
        const int64_t start_mhz = sweep_start_frequency_mhz();
        const int64_t end_mhz = sweep_end_frequency_mhz();
        const int64_t delta_mhz = end_mhz - start_mhz;
        const int64_t sweep_time = sweep_ms;
        const int64_t integral_millihz_ms =
            start_mhz * sweep_time +
            (delta_mhz * sweep_time * sweep_time) /
                (INT64_C(2) * static_cast<int64_t>(duration_ms));
        const int64_t end_integral_millihz_ms =
            start_mhz * static_cast<int64_t>(duration_ms) +
            (delta_mhz * static_cast<int64_t>(duration_ms)) / INT64_C(2);
        const int64_t hold_integral_millihz_ms =
            end_integral_millihz_ms +
            end_mhz * static_cast<int64_t>(elapsed_ms - sweep_ms);
        const int64_t phase_integral =
            (elapsed_ms < duration_ms) ? integral_millihz_ms : hold_integral_millihz_ms;

        frequency_mhz = static_cast<uint16_t>(
            start_mhz + (delta_mhz * static_cast<int64_t>(sweep_ms)) /
                            static_cast<int64_t>(duration_ms));
        phase = phase_from_millihz_ms(static_cast<uint64_t>(phase_integral));
    }
    else
    {
        frequency_mhz = fixed_test_frequency_mhz(mode);
        phase =
            phase_from_millihz_ms(static_cast<uint64_t>(frequency_mhz) * elapsed_ms);
    }

    const int32_t center = test_center_mm_x100(mode);
    const int32_t amplitude = test_amplitude_mm_x100(mode);
    int32_t offset = 0;
    if (mode == TestMode::Step)
    {
        offset = (phase < 32768U) ? -amplitude : amplitude;
    }
    else
    {
        offset = static_cast<int32_t>(
            (static_cast<int64_t>(amplitude) * foc_sin_q15(phase)) / 32767L);
    }

    s_position.test_phase = phase;
    s_position.test_frequency_mhz = frequency_mhz;
    const int64_t target = static_cast<int64_t>(center) + static_cast<int64_t>(offset);
    s_position.test_target_mm_x100 =
        static_cast<int32_t>(clamp<int64_t>(target,
                                            static_cast<int64_t>(INT32_MIN),
                                            static_cast<int64_t>(INT32_MAX)));
    s_position.sine_active = legacy_sine_requested(mode);
    s_position.sine_phase = s_position.sine_active ? phase : 0U;
    s_position.sine_target_mm_x100 =
        s_position.sine_active ? s_position.test_target_mm_x100 : 0;
    return s_position.test_target_mm_x100;
}

int32_t clamp_position_target_mm_x100(int32_t target) noexcept
{
    const int32_t max_mm_x100 = travel_mm_x100();
    if (max_mm_x100 <= 0)
    {
        return 0;
    }
    return clamp<int32_t>(target, 0, max_mm_x100);
}

int32_t clamp_position_target_counts(int32_t target) noexcept
{
    const int32_t max_counts = travel_counts();
    if (max_counts <= 0)
    {
        return 0;
    }
    return clamp<int32_t>(target, 0, max_counts);
}

void command_motor_position(int32_t absolute_ref,
                            int16_t speed_limit_rpm,
                            int16_t iq_limit,
                            int16_t kp,
                            uint8_t err_shift,
                            int32_t deadband_counts) noexcept
{
    g_mc_cmd.enable = 1U;
    g_mc_cmd.control_mode = kMotorModePosition;
    g_mc_cmd.id_ref = 0;
    g_mc_cmd.iq_ref = 0;
    g_mc_cmd.speed_ref = 0;
    g_mc_cmd.speed_ref_rpm = 0;
    g_mc_cmd.iq_limit = iq_limit;
    g_mc_cmd.position_ref = absolute_ref;
    g_mc_cmd.position_kp = kp;
    g_mc_cmd.position_err_shift = err_shift;
    g_mc_cmd.position_speed_limit_rpm = speed_limit_rpm;
    g_mc_cmd.position_deadband_counts = deadband_counts;
    g_mc_cmd.elec_zero_trim = kElecZeroTrim;
    s_axis_speed_cmd_rpm = 0;
}

bool home_enabled() noexcept
{
    return g_screw_axis_cmd.home_enable != 0U;
}

bool stop_requested() noexcept
{
    return g_screw_axis_cmd.stop != 0U;
}

bool home_busy() noexcept
{
    return (s_home.state != HomeState::Idle) && (s_home.state != HomeState::Done) &&
           (s_home.state != HomeState::Fault);
}

bool motor_fault_seen() noexcept
{
    // 底层 MotorControl fault 优先级最高，轴层只传播为 ScrewAxisFault_Motor。
    return (g_motor.runtime.state == MC_STATE_FAULT) || (g_motor.runtime.fault != 0U);
}

void set_home_fault(uint8_t reason) noexcept
{
    s_home.state = HomeState::Fault;
    s_axis_state = AxisState::Fault;
    s_home.fault_seen = true;
    s_home.fault_reason = reason;
    s_position.active = false;
    s_position.sine_active = false;
    s_position.test_active = false;
    s_position.test_mode = TestMode::Off;
    s_position.test_target_mm_x100 = 0;
    s_position.test_phase = 0U;
    s_position.test_frequency_mhz = 0U;
    g_screw_axis_cmd.home_enable = 0U;
    g_screw_axis_cmd.position_enable = 0U;
    g_screw_axis_cmd.sine_enable = 0U;
    g_screw_axis_cmd.test_enable = 0U;
}

void reset_stall_timer() noexcept
{
    s_home.stall_start_ms = 0U;
    s_home.stall_clear_start_ms = 0U;
}

void start_home(uint32_t now_ms) noexcept
{
    /*
     * 启动瞬间锁存命令参数。
     * 这样调试时可以先写多个 home_* 参数，最后写 home_enable=1 触发。
     */
    s_home.fast_speed_rpm = normalize_retract_speed(g_screw_axis_cmd.home_speed_rpm,
                                                    HomeConfig::fast_retract_rpm);
    s_home.extend_speed_rpm = HomeConfig::extend_rpm;
    s_home.slow_speed_rpm =
        normalize_retract_speed(g_screw_axis_cmd.home_slow_speed_rpm,
                                HomeConfig::slow_retract_rpm);
    s_home.backoff_speed_rpm =
        normalize_extend_speed(g_screw_axis_cmd.home_backoff_speed_rpm,
                               HomeConfig::backoff_rpm);
    s_home.timeout_ms = (g_screw_axis_cmd.home_timeout_ms == 0U)
                            ? HomeConfig::timeout_ms
                            : clamp<uint16_t>(g_screw_axis_cmd.home_timeout_ms,
                                              1000U,
                                              HomeConfig::timeout_ms);
    s_home.iq_limit = (g_screw_axis_cmd.home_iq_limit == 0)
                          ? HomeConfig::iq_limit
                          : abs_i16(g_screw_axis_cmd.home_iq_limit);

    s_home.phase_start_ms = now_ms;
    reset_stall_timer();
    s_home.fast_retract_hits = 0U;
    s_home.homed = false;
    s_home.outer_seen = false;
    s_home.fault_seen = false;
    s_home.fault_reason = ScrewAxisFault_None;
    s_home.zero_encoder_pos = 0;
    s_home.outer_encoder_pos = 0;
    s_position.active = false;
    s_position.target_mm_x100 = 0;
    s_position.sine_active = false;
    s_position.sine_target_mm_x100 = 0;
    s_position.sine_phase = 0U;
    s_position.test_active = false;
    s_position.test_target_mm_x100 = 0;
    s_position.test_phase = 0U;
    s_position.test_frequency_mhz = 0U;
    s_position.test_mode = TestMode::Off;
    s_position.target_counts = 0;
    s_position.absolute_position_ref = 0;
    s_home.state = HomeState::SeekOuter;
    s_axis_state = AxisState::Homing;
    s_axis_speed_cmd_rpm = 0;
    stop_motor(0U);
}

bool home_stalled_for(uint32_t now_ms, uint32_t hold_ms) noexcept
{
    /*
     * 机械端点判断不是看“速度为 0”一个条件。
     * 只有速度接近 0，同时速度环已经把 iq_ref 推到接近限幅，并持续 hold_ms，
     * 才认为已经顶到端点。
     */
    const int16_t iq_threshold =
        (s_home.iq_limit > HomeConfig::stall_iq_margin)
            ? static_cast<int16_t>(s_home.iq_limit - HomeConfig::stall_iq_margin)
            : 0;
    const bool stalled =
        (abs_i16(MotorControl_InternalSpeedCountsToRpm(g_motor.speed.feedback)) <=
         HomeConfig::stall_speed_rpm) &&
        (abs_i16(g_motor.speed.iq_ref.value) >= iq_threshold);

    if (!stalled)
    {
        /*
         * 已经开始堵转计时后，允许短暂速度尖峰。
         * 只有连续 stall_clear_hold_ms 都不满足堵转条件，才清掉计时。
         */
        if (s_home.stall_start_ms == 0U)
        {
            return false;
        }

        if (s_home.stall_clear_start_ms == 0U)
        {
            s_home.stall_clear_start_ms = now_ms;
        }

        if ((now_ms - s_home.stall_clear_start_ms) >= HomeConfig::stall_clear_hold_ms)
        {
            reset_stall_timer();
        }
        return false;
    }

    s_home.stall_clear_start_ms = 0U;
    if (s_home.stall_start_ms == 0U)
    {
        s_home.stall_start_ms = now_ms;
    }

    return (now_ms - s_home.stall_start_ms) >= hold_ms;
}

void update_home() noexcept
{
    const uint32_t now_ms = app_millis();
    const uint32_t elapsed_ms = now_ms - s_home.phase_start_ms;

    if (motor_fault_seen())
    {
        set_home_fault(ScrewAxisFault_Motor);
    }

    switch (s_home.state)
    {
    case HomeState::Idle:
        // Idle 等待 home_enable，不会自动重复回零。
        if (home_enabled())
        {
            start_home(now_ms);
        }
        break;

    case HomeState::SeekOuter:
        // 外伸到最大端。当前回零会执行两次“外伸到最大 -> 快速收回到最小”的粗来回。
        command_home_speed(s_home.extend_speed_rpm);
        if (elapsed_ms > s_home.timeout_ms)
        {
            set_home_fault(ScrewAxisFault_HomeSeekOuterTimeout);
        }
        else if (home_stalled_for(now_ms, HomeConfig::fast_stall_hold_ms))
        {
            s_home.outer_seen = true;
            s_home.outer_encoder_pos = g_motor.encoder.pos;
            s_home.state = HomeState::FastRetract;
            s_home.phase_start_ms = now_ms;
            reset_stall_timer();
        }
        break;

    case HomeState::FastRetract:
        // 负向快速收回到最小端。第二次收回后才进入精找零流程。
        command_home_speed(s_home.fast_speed_rpm);
        if (elapsed_ms > s_home.timeout_ms)
        {
            set_home_fault(ScrewAxisFault_HomeFastRetractTimeout);
        }
        else if (home_stalled_for(now_ms, HomeConfig::fast_stall_hold_ms))
        {
            if (s_home.fast_retract_hits < UINT8_MAX)
            {
                s_home.fast_retract_hits++;
            }

            s_home.state = (s_home.fast_retract_hits < HomeConfig::coarse_retract_hits)
                               ? HomeState::SeekOuter
                               : HomeState::FastBackoff;
            s_home.phase_start_ms = now_ms;
            reset_stall_timer();
        }
        break;

    case HomeState::FastBackoff:
        // 两次粗收回后，正向退离内端，释放机械压紧。
        command_home_speed(s_home.backoff_speed_rpm);
        if (elapsed_ms >= HomeConfig::fast_backoff_ms)
        {
            s_home.state = HomeState::SlowRetract;
            s_home.phase_start_ms = now_ms;
            reset_stall_timer();
        }
        break;

    case HomeState::SlowRetract:
        // 低速负向精找内端。这个阶段决定最终零位重复性。
        command_home_speed(s_home.slow_speed_rpm);
        if (elapsed_ms > s_home.timeout_ms)
        {
            set_home_fault(ScrewAxisFault_HomeSlowRetractTimeout);
        }
        else if (home_stalled_for(now_ms, HomeConfig::slow_stall_hold_ms))
        {
            // 记录内端零位。之后 pos_counts 都用当前 g_motor.encoder.pos 减这个快照。
            s_home.zero_encoder_pos = g_motor.encoder.pos;
            s_home.state = HomeState::FinalBackoff;
            s_home.phase_start_ms = now_ms;
            reset_stall_timer();
        }
        break;

    case HomeState::FinalBackoff:
        // 零位已记录，再正向退离一小段，避免长期顶住机械端点。
        command_home_speed(s_home.backoff_speed_rpm);
        if (elapsed_ms >= HomeConfig::final_backoff_ms)
        {
            s_home.homed = true;
            s_home.state = HomeState::Done;
            s_axis_state = AxisState::Ready;
            g_screw_axis_cmd.home_enable = 0U;
            stop_motor(0U);
        }
        break;

    case HomeState::Done:
        // Done 状态保持电机停止，同时允许 home_enable 重新触发回零。
        stop_motor(0U);
        s_axis_state = s_home.homed ? AxisState::Ready : AxisState::Idle;
        if (home_enabled())
        {
            start_home(now_ms);
        }
        break;

    case HomeState::Fault:
        // Fault 状态下不再输出速度命令，等待用户处理底层 fault 或复位/重新初始化。
        s_axis_state = AxisState::Fault;
        stop_motor(0U);
        break;
    }

    publish_status();
}

void update_position_command() noexcept
{
    const TestMode test_mode = requested_test_mode();
    const bool enabled = position_enabled();
    const bool test = test_mode != TestMode::Off;

    if (!enabled && !test)
    {
        if (s_position.active)
        {
            s_position.active = false;
            s_position.sine_active = false;
            s_position.test_active = false;
            s_position.test_mode = TestMode::Off;
            s_position.test_target_mm_x100 = 0;
            s_position.test_phase = 0U;
            s_position.test_frequency_mhz = 0U;
            stop_motor(0U);
        }
        return;
    }

    if (!s_home.homed)
    {
        s_position.active = false;
        return;
    }

    if (travel_counts() <= 0)
    {
        s_position.active = false;
        return;
    }

    /*
     * position_enable 为 1 时直接用 target_mm_x100；测试使能后由轨迹发生器
     * 覆盖目标。轨迹相位从 ADC 派生的毫秒时基计算，不依赖主循环调用次数。
     */
    const int32_t commanded_target_mm_x100 = g_screw_axis_cmd.target_mm_x100;
    const int32_t target_mm_x100 =
        test ? update_test_target_mm_x100(test_mode, app_millis())
             : commanded_target_mm_x100;
    if (!test)
    {
        s_position.sine_active = false;
        s_position.test_active = false;
        s_position.test_mode = TestMode::Off;
        s_position.test_target_mm_x100 = 0;
        s_position.test_phase = 0U;
        s_position.test_frequency_mhz = 0U;
    }

    s_position.target_mm_x100 = clamp_position_target_mm_x100(target_mm_x100);
    s_position.target_counts =
        clamp_position_target_counts(mm_x100_to_counts(s_position.target_mm_x100));
    s_position.absolute_position_ref =
        s_home.zero_encoder_pos + s_position.target_counts;
    const int16_t speed_limit_rpm = g_screw_axis_cmd.position_speed_limit_rpm;
    const int16_t iq_limit = g_screw_axis_cmd.position_iq_limit;
    const int16_t kp = g_screw_axis_cmd.position_kp;
    const uint8_t err_shift = g_screw_axis_cmd.position_err_shift;
    const int32_t deadband_counts = g_screw_axis_cmd.position_deadband_counts;
    s_position.speed_limit_rpm = normalize_position_speed_limit(speed_limit_rpm);
    s_position.iq_limit = normalize_position_iq_limit(iq_limit);
    s_position.kp = normalize_position_kp(kp);
    s_position.err_shift = normalize_position_err_shift(err_shift);
    s_position.deadband_counts = normalize_position_deadband_counts(deadband_counts);
    s_position.active = true;

    command_motor_position(s_position.absolute_position_ref,
                           s_position.speed_limit_rpm,
                           s_position.iq_limit,
                           s_position.kp,
                           s_position.err_shift,
                           s_position.deadband_counts);

    if (!enabled && (g_motor.runtime.mode == MC_MODE_POSITION) &&
        (g_motor.position.target == s_position.absolute_position_ref) &&
        (g_motor.position.at_target != 0U))
    {
        s_position.active = false;
    }
}

} // namespace

volatile ScrewAxisCommand_t g_screw_axis_cmd = {
    0U,
    0U,
    HomeConfig::fast_retract_rpm,
    HomeConfig::slow_retract_rpm,
    HomeConfig::backoff_rpm,
    HomeConfig::timeout_ms,
    HomeConfig::iq_limit,
    0U,
    0U,
    0,
    PositionConfig::sine_center_mm_x100,
    PositionConfig::sine_amplitude_mm_x100,
    PositionConfig::sine_period_ms,
    PositionConfig::speed_limit_rpm,
    PositionConfig::iq_limit,
    PositionConfig::kp,
    PositionConfig::err_shift,
    PositionConfig::deadband_counts,
    0U,
    ScrewAxisTestMode_Off,
    PositionConfig::test_center_mm_x100,
    PositionConfig::test_amplitude_mm_x100,
    PositionConfig::test_frequency_mhz,
    PositionConfig::test_sweep_start_frequency_mhz,
    PositionConfig::test_sweep_end_frequency_mhz,
    PositionConfig::test_sweep_duration_ms,
};

volatile ScrewAxisWatch_t g_screw_axis_watch;

extern "C" void ScrewAxis_Init(void)
{
    s_adc_samples = 0U;
    s_home = HomeRuntime{};
    s_position = PositionRuntime{};
    s_axis_state = AxisState::Idle;
    s_axis_speed_cmd_rpm = 0;

    publish_status();
    stop_motor(0U);
}

extern "C" void ScrewAxis_Run(void)
{
    if (stop_requested())
    {
        s_home.state = s_home.homed ? HomeState::Done : HomeState::Idle;
        s_axis_state = s_home.homed ? AxisState::Ready : AxisState::Idle;
        s_position.active = false;
        s_position.sine_active = false;
        s_position.test_active = false;
        s_position.test_mode = TestMode::Off;
        g_screw_axis_cmd.stop = 0U;
        g_screw_axis_cmd.home_enable = 0U;
        g_screw_axis_cmd.position_enable = 0U;
        g_screw_axis_cmd.sine_enable = 0U;
        g_screw_axis_cmd.test_enable = 0U;
        stop_motor(0U);
        publish_status();
        return;
    }

    if (home_busy() || home_enabled())
    {
        update_home();
        return;
    }

    if (s_home.state == HomeState::Fault)
    {
        if (s_home.homed &&
            (position_enabled() || (requested_test_mode() != TestMode::Off)))
        {
            s_home.state = HomeState::Done;
            s_axis_state = AxisState::Ready;
            s_home.fault_seen = false;
            s_home.fault_reason = ScrewAxisFault_None;
            update_position_command();
            publish_status();
            return;
        }

        s_axis_state = AxisState::Fault;
        stop_motor(0U);
        publish_status();
        return;
    }

    if (position_enabled() || (requested_test_mode() != TestMode::Off) ||
        s_position.active)
    {
        update_position_command();
        publish_status();
        return;
    }

    /*
     * 空闲/Ready 时不要持续覆盖 g_mc_cmd。
     * 回零、stop、fault 和回零完成瞬间会主动关闭 MotorControl；之后把命令邮箱
     * 让出来，方便 Ozone 或后续轴层位置命令直接测试 MotorControl 的 Position 模式。
     */
    s_axis_state = s_home.homed ? AxisState::Ready : AxisState::Idle;
    publish_status();
}

extern "C" void ScrewAxis_OnAdcSample(void)
{
    s_adc_samples++;
}

#include "screw_axis.h"

#include <stdint.h>

#include "BoardConfig.h"
#include "MotorControl.h"

#include "clamp.hpp"

/*
 * ScrewAxis 当前只做一件事：上电/调试时用速度模式完成螺杆回零。
 * 对外保持 C ABI，内部用少量 C++ 类型约束状态和值域。
 */
namespace
{

using cms32::support::clamp;

// MotorControl 仍是 C 接口，这里只写入它已有的速度模式命令。
constexpr uint8_t kMotorModeSpeed = 2U;
constexpr int16_t kSpeedMaxRpm = 500;
constexpr int16_t kIqLimitMax = 16;

// 默认参数也是 g_screw_home_cmd 的上电初值。
constexpr int16_t kDefaultFastSpeedRpm = -300;
constexpr int16_t kDefaultSlowSpeedRpm = -50;
constexpr int16_t kDefaultBackoffSpeedRpm = 120;
constexpr int16_t kDefaultIqLimit = 12;
constexpr uint16_t kHomeTimeoutMs = 12000U;

// 通过“速度接近 0 且 q 轴命令接近限幅”判断机械端点。
constexpr int16_t kStallSpeedRpm = 8;
constexpr int16_t kStallIqMargin = 2;
constexpr uint32_t kFastStallHoldMs = 250U;
constexpr uint32_t kSlowStallHoldMs = 600U;
constexpr uint32_t kFastBackoffMs = 700U;
constexpr uint32_t kFinalBackoffMs = 800U;

constexpr int16_t kElecZeroTrim = 0;

enum class HomeState : uint8_t
{
    Idle = ScrewHomeState_Idle,
    FastRetract = ScrewHomeState_FastRetract,
    FastBackoff = ScrewHomeState_FastBackoff,
    SlowRetract = ScrewHomeState_SlowRetract,
    FinalBackoff = ScrewHomeState_FinalBackoff,
    Done = ScrewHomeState_Done,
    Fault = ScrewHomeState_Fault,
};

struct HomeRuntime
{
    HomeState state{HomeState::Idle};
    uint16_t last_start_seq{0U};
    uint32_t phase_start_ms{0U};
    uint32_t stall_start_ms{0U};
    int16_t fast_speed_rpm{kDefaultFastSpeedRpm};
    int16_t slow_speed_rpm{kDefaultSlowSpeedRpm};
    int16_t backoff_speed_rpm{kDefaultBackoffSpeedRpm};
    int16_t iq_limit{kDefaultIqLimit};
    uint16_t timeout_ms{kHomeTimeoutMs};
    bool homed{false};
    bool fault_seen{false};
    int32_t zero_encoder_pos{0};
};

// s_adc_samples 只在 ADC 有效控制采样后递增，避免空转主循环影响时间基。
volatile uint32_t s_adc_samples;
HomeRuntime s_home;

constexpr int16_t abs_i16(int16_t value) noexcept
{
    if (value >= 0)
    {
        return value;
    }
    if (value == INT16_MIN)
    {
        return static_cast<int16_t>(INT16_MAX);
    }
    return static_cast<int16_t>(-value);
}

uint32_t app_millis() noexcept
{
    return s_adc_samples / (PWM_FREQ_HZ / 1000U);
}

int16_t normalize_retract_speed(int16_t value, int16_t fallback) noexcept
{
    int16_t speed = abs_i16(value);
    if (speed == 0)
    {
        speed = abs_i16(fallback);
    }
    speed = clamp<int16_t>(speed, 1, kSpeedMaxRpm);
    return static_cast<int16_t>(-speed);
}

int16_t normalize_backoff_speed(int16_t value) noexcept
{
    int16_t speed = abs_i16(value);
    if (speed == 0)
    {
        speed = kDefaultBackoffSpeedRpm;
    }
    return clamp<int16_t>(speed, 1, kSpeedMaxRpm);
}

void publish_status() noexcept
{
    const bool busy = (s_home.state != HomeState::Idle) && (s_home.state != HomeState::Done) &&
                      (s_home.state != HomeState::Fault);

    g_screw_home_watch.busy = busy ? 1U : 0U;
    g_screw_home_watch.homed = s_home.homed ? 1U : 0U;
    g_screw_home_watch.fault_seen = s_home.fault_seen ? 1U : 0U;
    g_screw_home_watch.state = static_cast<uint8_t>(s_home.state);
    g_screw_home_watch.active_seq = s_home.last_start_seq;
    g_screw_home_watch.zero_encoder_pos = s_home.zero_encoder_pos;
    g_screw_home_watch.pos_counts =
        s_home.homed ? (g_motor_status.encoder_pos - s_home.zero_encoder_pos) : 0;
}

void command_speed(int16_t speed_rpm) noexcept
{
    g_motor_command.enable = 1U;
    g_motor_command.control_mode = kMotorModeSpeed;
    g_motor_command.id_ref = 0;
    g_motor_command.iq_ref = 0;
    g_motor_command.speed_ref = 0;
    g_motor_command.speed_ref_rpm = speed_rpm;
    g_motor_command.iq_limit = s_home.iq_limit;
    g_motor_command.elec_zero_trim = kElecZeroTrim;
}

void stop_motor(uint8_t keep_enabled) noexcept
{
    g_motor_command.enable = keep_enabled;
    g_motor_command.control_mode = kMotorModeSpeed;
    g_motor_command.id_ref = 0;
    g_motor_command.iq_ref = 0;
    g_motor_command.speed_ref = 0;
    g_motor_command.speed_ref_rpm = 0;
}

void start_home(uint32_t now_ms) noexcept
{
    // 参数在启动瞬间锁存，避免 Ozone/watch 运行中修改造成状态机跳变。
    s_home.last_start_seq = g_screw_home_cmd.start_seq;
    s_home.fast_speed_rpm =
        normalize_retract_speed(g_screw_home_cmd.speed_rpm, kDefaultFastSpeedRpm);
    s_home.slow_speed_rpm =
        normalize_retract_speed(g_screw_home_cmd.slow_speed_rpm, kDefaultSlowSpeedRpm);
    s_home.backoff_speed_rpm = normalize_backoff_speed(g_screw_home_cmd.backoff_speed_rpm);
    s_home.timeout_ms = clamp<uint16_t>(g_screw_home_cmd.timeout_ms, 1000U, kHomeTimeoutMs);
    s_home.iq_limit = clamp<int16_t>(g_screw_home_cmd.iq_limit, 1, kIqLimitMax);

    s_home.phase_start_ms = now_ms;
    s_home.stall_start_ms = 0U;
    s_home.homed = false;
    s_home.fault_seen = false;
    s_home.zero_encoder_pos = 0;
    s_home.state = HomeState::FastRetract;
}

bool stalled_for(uint32_t now_ms, uint32_t hold_ms) noexcept
{
    const int16_t iq_threshold =
        clamp<int16_t>(static_cast<int16_t>(s_home.iq_limit - kStallIqMargin), 0, kIqLimitMax);
    const bool stalled = (abs_i16(g_motor_status.speed_fb_rpm) <= kStallSpeedRpm) &&
                         (abs_i16(g_motor_status.speed_iq_cmd) >= iq_threshold);

    if (!stalled)
    {
        s_home.stall_start_ms = 0U;
        return false;
    }

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

    if (g_screw_home_cmd.stop != 0U)
    {
        s_home.state = HomeState::Idle;
        stop_motor(0U);
        publish_status();
        return;
    }

    if ((g_motor_status.state == 4U) || (g_motor_status.fault_reason != 0U))
    {
        s_home.state = HomeState::Fault;
        s_home.fault_seen = true;
    }

    switch (s_home.state)
    {
    case HomeState::Idle:
        // start_seq 是边沿触发命令，避免上电后重复开始。
        if (g_screw_home_cmd.start_seq != s_home.last_start_seq)
        {
            start_home(now_ms);
        }
        break;

    case HomeState::FastRetract:
        // 第一次快速撞端点，只用于粗定位。
        command_speed(s_home.fast_speed_rpm);
        if (elapsed_ms > s_home.timeout_ms)
        {
            s_home.state = HomeState::Fault;
            s_home.fault_seen = true;
        }
        else if (stalled_for(now_ms, kFastStallHoldMs))
        {
            s_home.state = HomeState::FastBackoff;
            s_home.phase_start_ms = now_ms;
            s_home.stall_start_ms = 0U;
        }
        break;

    case HomeState::FastBackoff:
        // 退离端点后再低速靠近，可以减少最终零位受冲击影响。
        command_speed(s_home.backoff_speed_rpm);
        if (elapsed_ms >= kFastBackoffMs)
        {
            s_home.state = HomeState::SlowRetract;
            s_home.phase_start_ms = now_ms;
            s_home.stall_start_ms = 0U;
        }
        break;

    case HomeState::SlowRetract:
        // 第二次低速撞端点，确认后记录编码器零位。
        command_speed(s_home.slow_speed_rpm);
        if (elapsed_ms > s_home.timeout_ms)
        {
            s_home.state = HomeState::Fault;
            s_home.fault_seen = true;
        }
        else if (stalled_for(now_ms, kSlowStallHoldMs))
        {
            s_home.homed = true;
            s_home.zero_encoder_pos = g_motor_status.encoder_pos;
            s_home.state = HomeState::FinalBackoff;
            s_home.phase_start_ms = now_ms;
            s_home.stall_start_ms = 0U;
        }
        break;

    case HomeState::FinalBackoff:
        // 零位已记录，最后退离端点，避免长期顶住机械端。
        command_speed(s_home.backoff_speed_rpm);
        if (elapsed_ms >= kFinalBackoffMs)
        {
            s_home.state = HomeState::Done;
            stop_motor(0U);
        }
        break;

    case HomeState::Done:
        stop_motor(0U);
        if (g_screw_home_cmd.start_seq != s_home.last_start_seq)
        {
            start_home(now_ms);
        }
        break;

    case HomeState::Fault:
        stop_motor(0U);
        break;
    }

    publish_status();
}

} // namespace

volatile ScrewHomeCommand_t g_screw_home_cmd = {
    0U,
    0U,
    kDefaultFastSpeedRpm,
    kDefaultSlowSpeedRpm,
    kDefaultBackoffSpeedRpm,
    kHomeTimeoutMs,
    kDefaultIqLimit,
};

volatile ScrewHomeWatch_t g_screw_home_watch;

extern "C" void ScrewAxis_Init(void)
{
    s_adc_samples = 0U;
    s_home = HomeRuntime{};
    s_home.last_start_seq = g_screw_home_cmd.start_seq;

    publish_status();
    stop_motor(0U);
}

extern "C" void ScrewAxis_Run(void)
{
    update_home();
}

extern "C" void ScrewAxis_OnAdcSample(void)
{
    s_adc_samples++;
}

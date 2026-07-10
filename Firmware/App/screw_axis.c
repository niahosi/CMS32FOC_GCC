#include "screw_axis.h"

#include "Config.h"
#include "MotorControl.h"

#define SCREW_MC_MODE_SPEED 2U

#define SCREW_JOG_DEFAULT_SPEED_RPM 30
#define SCREW_JOG_DEFAULT_IQ_LIMIT 12
#define SCREW_JOG_DEFAULT_RUN_MS 500U
#define SCREW_JOG_MAX_SPEED_RPM 500
#define SCREW_JOG_MAX_IQ_LIMIT 16
#define SCREW_JOG_MAX_RUN_MS 3000U
#define SCREW_JOG_PAUSE_MS 1000U
#define SCREW_JOG_MAX_REPEAT_PAIRS 20U

#define SCREW_HOME_DEFAULT_FAST_SPEED_RPM (-300)
#define SCREW_HOME_DEFAULT_SLOW_SPEED_RPM (-50)
#define SCREW_HOME_BACKOFF_SPEED_RPM 120
#define SCREW_HOME_DEFAULT_IQ_LIMIT 12
#define SCREW_HOME_TIMEOUT_MS 12000U
#define SCREW_HOME_STALL_SPEED_RPM 8
#define SCREW_HOME_STALL_IQ_MARGIN 2
#define SCREW_HOME_FAST_STALL_HOLD_MS 250U
#define SCREW_HOME_SLOW_STALL_HOLD_MS 600U
#define SCREW_HOME_FAST_BACKOFF_MS 700U
#define SCREW_HOME_FINAL_BACKOFF_MS 800U

#define SCREW_TRAVEL_UM 10000L
#define SCREW_COUNTS_PER_MM 524288L
#define SCREW_POS_DEFAULT_MAX_SPEED_RPM 150
#define SCREW_POS_DEFAULT_IQ_LIMIT 12
#define SCREW_POS_DEFAULT_KP_RPM_PER_MM 100
#define SCREW_POS_DEFAULT_DEADBAND_UM 30
#define SCREW_ELEC_ZERO_TRIM 0

typedef struct
{
    uint8_t state;
    uint16_t last_start_seq;
    uint32_t phase_start_ms;
    uint32_t stall_start_ms;
    int16_t active_speed_rpm;
    int16_t active_slow_speed_rpm;
    int16_t active_backoff_speed_rpm;
    int16_t active_iq_limit;
    uint16_t active_timeout_ms;
} ScrewHomeState_t;

typedef struct
{
    uint8_t phase;
    uint16_t last_start_seq;
    uint32_t phase_start_ms;
    uint16_t active_run_ms;
    uint16_t active_repeat_pairs;
    uint16_t active_leg_index;
    int16_t active_speed_rpm;
    int16_t active_base_speed_rpm;
    int16_t active_iq_limit;
} ScrewJogState_t;

volatile ScrewJogCommand_t g_screw_jog_cmd = {
    .start_seq = 0U,
    .stop = 0U,
    .speed_rpm = SCREW_JOG_DEFAULT_SPEED_RPM,
    .run_ms = SCREW_JOG_DEFAULT_RUN_MS,
    .iq_limit = SCREW_JOG_DEFAULT_IQ_LIMIT,
    .repeat_pairs = 0U,
};

volatile ScrewJogWatch_t g_screw_jog_watch;

volatile ScrewHomeCommand_t g_screw_home_cmd = {
    .start_seq = 0U,
    .stop = 0U,
    .speed_rpm = SCREW_HOME_DEFAULT_FAST_SPEED_RPM,
    .slow_speed_rpm = SCREW_HOME_DEFAULT_SLOW_SPEED_RPM,
    .backoff_speed_rpm = SCREW_HOME_BACKOFF_SPEED_RPM,
    .timeout_ms = SCREW_HOME_TIMEOUT_MS,
    .iq_limit = SCREW_HOME_DEFAULT_IQ_LIMIT,
};

volatile ScrewHomeWatch_t g_screw_home_watch;

volatile ScrewPositionCommand_t g_screw_pos_cmd = {
    .enable = 0U,
    .stop = 0U,
    .target_um = 0,
    .max_speed_rpm = SCREW_POS_DEFAULT_MAX_SPEED_RPM,
    .kp_rpm_per_mm = SCREW_POS_DEFAULT_KP_RPM_PER_MM,
    .deadband_um = SCREW_POS_DEFAULT_DEADBAND_UM,
    .iq_limit = SCREW_POS_DEFAULT_IQ_LIMIT,
};

volatile ScrewPositionWatch_t g_screw_pos_watch;

static volatile uint32_t s_adc_samples;
static ScrewHomeState_t s_home;
static ScrewJogState_t s_jog;

static uint32_t app_millis(void);
static void home_start(uint32_t now_ms);
static uint8_t home_update(void);
static uint8_t position_update(void);
static void jog_update(void);
static int16_t abs_s16(int16_t value);
static int16_t clamp_s16(int16_t value, int16_t min_value, int16_t max_value);
static uint16_t clamp_u16(uint16_t value, uint16_t min_value, uint16_t max_value);
static int32_t clamp_s32(int32_t value, int32_t min_value, int32_t max_value);
static int32_t counts_to_um(int32_t counts);
static int32_t um_to_counts(int32_t um);
static void apply_common_speed_command(int16_t iq_limit);
static void stop_speed_command(uint8_t keep_enabled);

void ScrewAxis_Init(void)
{
    s_adc_samples = 0U;
    s_home = (ScrewHomeState_t){
        .state = SCREW_HOME_STATE_IDLE,
        .last_start_seq = g_screw_home_cmd.start_seq,
    };
    s_jog = (ScrewJogState_t){
        .phase = SCREW_JOG_PHASE_IDLE,
        .last_start_seq = g_screw_jog_cmd.start_seq,
    };

    g_screw_jog_watch = (ScrewJogWatch_t){0};
    g_screw_home_watch = (ScrewHomeWatch_t){0};
    g_screw_home_watch.state = SCREW_HOME_STATE_IDLE;
    g_screw_pos_watch = (ScrewPositionWatch_t){0};
    stop_speed_command(0U);
}

void ScrewAxis_Run(void)
{
    if (home_update() == 0U)
    {
        if (position_update() == 0U)
        {
            jog_update();
        }
    }
}

void ScrewAxis_OnAdcSample(void)
{
    s_adc_samples++;
}

static uint32_t app_millis(void)
{
    return s_adc_samples / (PWM_FREQ_HZ / 1000U);
}

static void home_start(uint32_t now_ms)
{
    s_home.last_start_seq = g_screw_home_cmd.start_seq;
    s_home.active_speed_rpm =
        clamp_s16(g_screw_home_cmd.speed_rpm, (int16_t)-SCREW_JOG_MAX_SPEED_RPM,
                  SCREW_JOG_MAX_SPEED_RPM);
    if (s_home.active_speed_rpm > 0)
    {
        s_home.active_speed_rpm = (int16_t)-s_home.active_speed_rpm;
    }
    if (s_home.active_speed_rpm == 0)
    {
        s_home.active_speed_rpm = SCREW_HOME_DEFAULT_FAST_SPEED_RPM;
    }

    s_home.active_slow_speed_rpm =
        clamp_s16(g_screw_home_cmd.slow_speed_rpm, (int16_t)-SCREW_JOG_MAX_SPEED_RPM,
                  SCREW_JOG_MAX_SPEED_RPM);
    if (s_home.active_slow_speed_rpm > 0)
    {
        s_home.active_slow_speed_rpm = (int16_t)-s_home.active_slow_speed_rpm;
    }
    if (s_home.active_slow_speed_rpm == 0)
    {
        s_home.active_slow_speed_rpm = SCREW_HOME_DEFAULT_SLOW_SPEED_RPM;
    }

    s_home.active_backoff_speed_rpm =
        clamp_s16(g_screw_home_cmd.backoff_speed_rpm, 1, SCREW_JOG_MAX_SPEED_RPM);
    s_home.active_timeout_ms =
        clamp_u16(g_screw_home_cmd.timeout_ms, 1000U, SCREW_HOME_TIMEOUT_MS);
    s_home.active_iq_limit =
        clamp_s16(g_screw_home_cmd.iq_limit, 1, SCREW_JOG_MAX_IQ_LIMIT);
    s_home.phase_start_ms = now_ms;
    s_home.stall_start_ms = 0U;
    s_home.state = SCREW_HOME_STATE_FAST_RETRACT;

    g_screw_home_watch.busy = 1U;
    g_screw_home_watch.homed = 0U;
    g_screw_home_watch.fault_seen = 0U;
    g_screw_home_watch.state = s_home.state;
    g_screw_home_watch.active_seq = s_home.last_start_seq;
    g_screw_home_watch.active_speed_rpm = s_home.active_speed_rpm;
    g_screw_home_watch.active_slow_speed_rpm = s_home.active_slow_speed_rpm;
    g_screw_home_watch.active_backoff_speed_rpm = s_home.active_backoff_speed_rpm;
    g_screw_home_watch.elapsed_ms = 0U;
    g_screw_home_watch.remaining_ms = s_home.active_timeout_ms;
    g_screw_home_watch.stall_elapsed_ms = 0U;
}

static uint8_t home_update(void)
{
    const uint32_t now_ms = app_millis();
    const uint32_t elapsed_ms = now_ms - s_home.phase_start_ms;
    const int16_t stall_iq_threshold =
        (int16_t)(s_home.active_iq_limit - SCREW_HOME_STALL_IQ_MARGIN);
    const uint8_t stall_condition =
        (uint8_t)((abs_s16(g_motor_watch.speed_fb_rpm) <= SCREW_HOME_STALL_SPEED_RPM) &&
                  (abs_s16(g_motor_watch.speed_iq_cmd) >= stall_iq_threshold));

    if (g_screw_home_watch.homed != 0U)
    {
        g_screw_home_watch.pos_counts =
            (int32_t)g_motor_watch.encoder_pos - g_screw_home_watch.zero_encoder_pos;
    }

    if (g_screw_home_cmd.stop != 0U)
    {
        s_home.state = SCREW_HOME_STATE_IDLE;
        stop_speed_command(0U);
        g_screw_home_watch.busy = 0U;
        g_screw_home_watch.state = s_home.state;
        return 1U;
    }

    if ((g_motor_watch.state == 4U) || (g_motor_watch.fault_reason != 0U))
    {
        s_home.state = SCREW_HOME_STATE_FAULT;
        g_screw_home_watch.fault_seen = 1U;
    }

    switch (s_home.state)
    {
        case SCREW_HOME_STATE_IDLE:
            g_screw_home_watch.busy = 0U;
            g_screw_home_watch.state = s_home.state;
            g_screw_home_watch.elapsed_ms = 0U;
            g_screw_home_watch.remaining_ms = 0U;
            g_screw_home_watch.stall_elapsed_ms = 0U;
            if (g_screw_home_cmd.start_seq == s_home.last_start_seq)
            {
                return 0U;
            }
            home_start(now_ms);
            return 1U;

        case SCREW_HOME_STATE_FAST_RETRACT:
            apply_common_speed_command(s_home.active_iq_limit);
            g_motor_cmd.enable = 1U;
            g_motor_cmd.speed_ref_rpm = s_home.active_speed_rpm;
            g_screw_home_watch.busy = 1U;
            g_screw_home_watch.state = s_home.state;
            g_screw_home_watch.elapsed_ms =
                (elapsed_ms > 65535U) ? 65535U : (uint16_t)elapsed_ms;
            g_screw_home_watch.remaining_ms =
                (elapsed_ms >= s_home.active_timeout_ms)
                    ? 0U
                    : (uint16_t)(s_home.active_timeout_ms - elapsed_ms);
            if (stall_condition != 0U)
            {
                if (s_home.stall_start_ms == 0U)
                {
                    s_home.stall_start_ms = now_ms;
                }
                g_screw_home_watch.stall_elapsed_ms =
                    (uint16_t)(now_ms - s_home.stall_start_ms);
            }
            else
            {
                s_home.stall_start_ms = 0U;
                g_screw_home_watch.stall_elapsed_ms = 0U;
            }

            if (g_screw_home_watch.stall_elapsed_ms >= SCREW_HOME_FAST_STALL_HOLD_MS)
            {
                s_home.phase_start_ms = now_ms;
                s_home.stall_start_ms = 0U;
                s_home.state = SCREW_HOME_STATE_FAST_BACKOFF;
            }
            else if (elapsed_ms >= s_home.active_timeout_ms)
            {
                s_home.state = SCREW_HOME_STATE_FAULT;
                g_screw_home_watch.fault_seen = 1U;
                stop_speed_command(0U);
            }
            return 1U;

        case SCREW_HOME_STATE_FAST_BACKOFF:
            apply_common_speed_command(s_home.active_iq_limit);
            g_motor_cmd.enable = 1U;
            g_motor_cmd.speed_ref_rpm = s_home.active_backoff_speed_rpm;
            g_screw_home_watch.busy = 1U;
            g_screw_home_watch.state = s_home.state;
            g_screw_home_watch.elapsed_ms =
                (elapsed_ms > 65535U) ? 65535U : (uint16_t)elapsed_ms;
            g_screw_home_watch.remaining_ms =
                (elapsed_ms >= SCREW_HOME_FAST_BACKOFF_MS)
                    ? 0U
                    : (uint16_t)(SCREW_HOME_FAST_BACKOFF_MS - elapsed_ms);
            if (elapsed_ms >= SCREW_HOME_FAST_BACKOFF_MS)
            {
                s_home.phase_start_ms = now_ms;
                s_home.stall_start_ms = 0U;
                s_home.state = SCREW_HOME_STATE_SLOW_RETRACT;
            }
            return 1U;

        case SCREW_HOME_STATE_SLOW_RETRACT:
            apply_common_speed_command(s_home.active_iq_limit);
            g_motor_cmd.enable = 1U;
            g_motor_cmd.speed_ref_rpm = s_home.active_slow_speed_rpm;
            g_screw_home_watch.busy = 1U;
            g_screw_home_watch.state = s_home.state;
            g_screw_home_watch.elapsed_ms =
                (elapsed_ms > 65535U) ? 65535U : (uint16_t)elapsed_ms;
            g_screw_home_watch.remaining_ms =
                (elapsed_ms >= s_home.active_timeout_ms)
                    ? 0U
                    : (uint16_t)(s_home.active_timeout_ms - elapsed_ms);
            if (stall_condition != 0U)
            {
                if (s_home.stall_start_ms == 0U)
                {
                    s_home.stall_start_ms = now_ms;
                }
                g_screw_home_watch.stall_elapsed_ms =
                    (uint16_t)(now_ms - s_home.stall_start_ms);
            }
            else
            {
                s_home.stall_start_ms = 0U;
                g_screw_home_watch.stall_elapsed_ms = 0U;
            }

            if (g_screw_home_watch.stall_elapsed_ms >= SCREW_HOME_SLOW_STALL_HOLD_MS)
            {
                g_screw_home_watch.zero_encoder_pos = g_motor_watch.encoder_pos;
                g_screw_home_watch.pos_counts = 0;
                g_screw_home_watch.homed = 1U;
                s_home.phase_start_ms = now_ms;
                s_home.state = SCREW_HOME_STATE_FINAL_BACKOFF;
            }
            else if (elapsed_ms >= s_home.active_timeout_ms)
            {
                s_home.state = SCREW_HOME_STATE_FAULT;
                g_screw_home_watch.fault_seen = 1U;
                stop_speed_command(0U);
            }
            return 1U;

        case SCREW_HOME_STATE_FINAL_BACKOFF:
            apply_common_speed_command(s_home.active_iq_limit);
            g_motor_cmd.enable = 1U;
            g_motor_cmd.speed_ref_rpm = s_home.active_backoff_speed_rpm;
            g_screw_home_watch.busy = 1U;
            g_screw_home_watch.state = s_home.state;
            g_screw_home_watch.elapsed_ms =
                (elapsed_ms > 65535U) ? 65535U : (uint16_t)elapsed_ms;
            g_screw_home_watch.remaining_ms =
                (elapsed_ms >= SCREW_HOME_FINAL_BACKOFF_MS)
                    ? 0U
                    : (uint16_t)(SCREW_HOME_FINAL_BACKOFF_MS - elapsed_ms);
            if (elapsed_ms >= SCREW_HOME_FINAL_BACKOFF_MS)
            {
                s_home.state = SCREW_HOME_STATE_DONE;
                stop_speed_command(0U);
            }
            return 1U;

        case SCREW_HOME_STATE_DONE:
            stop_speed_command(0U);
            g_screw_home_watch.busy = 0U;
            g_screw_home_watch.state = s_home.state;
            if (g_screw_home_cmd.start_seq != s_home.last_start_seq)
            {
                home_start(now_ms);
                return 1U;
            }
            return 0U;

        case SCREW_HOME_STATE_FAULT:
        default:
            stop_speed_command(0U);
            g_screw_home_watch.busy = 0U;
            g_screw_home_watch.state = s_home.state;
            return 1U;
    }
}

static uint8_t position_update(void)
{
    int32_t pos_counts;
    int32_t pos_um;
    int32_t target_um;
    int32_t error_um;
    int32_t speed_rpm;
    int16_t max_speed_rpm;
    int16_t kp_rpm_per_mm;
    int16_t iq_limit;
    uint16_t deadband_um;

    if (g_screw_pos_cmd.stop != 0U)
    {
        stop_speed_command(0U);
        g_screw_pos_watch.active = 0U;
        g_screw_pos_watch.speed_ref_rpm = 0;
        return 1U;
    }

    if (g_screw_pos_cmd.enable == 0U)
    {
        g_screw_pos_watch.active = 0U;
        g_screw_pos_watch.speed_ref_rpm = 0;
        return 0U;
    }

    if ((g_motor_watch.state == 4U) || (g_motor_watch.fault_reason != 0U))
    {
        stop_speed_command(0U);
        g_screw_pos_watch.active = 0U;
        g_screw_pos_watch.fault_seen = 1U;
        g_screw_pos_watch.speed_ref_rpm = 0;
        return 1U;
    }

    if (g_screw_home_watch.homed == 0U)
    {
        stop_speed_command(0U);
        g_screw_pos_watch.active = 0U;
        g_screw_pos_watch.not_homed = 1U;
        g_screw_pos_watch.speed_ref_rpm = 0;
        return 1U;
    }

    pos_counts = (int32_t)g_motor_watch.encoder_pos - g_screw_home_watch.zero_encoder_pos;
    pos_um = counts_to_um(pos_counts);
    target_um = clamp_s32(g_screw_pos_cmd.target_um, 0, SCREW_TRAVEL_UM);
    error_um = target_um - pos_um;
    max_speed_rpm =
        clamp_s16(g_screw_pos_cmd.max_speed_rpm, 1, SCREW_JOG_MAX_SPEED_RPM);
    kp_rpm_per_mm = clamp_s16(g_screw_pos_cmd.kp_rpm_per_mm, 1, 1000);
    iq_limit = clamp_s16(g_screw_pos_cmd.iq_limit, 1, SCREW_JOG_MAX_IQ_LIMIT);
    deadband_um = clamp_u16(g_screw_pos_cmd.deadband_um, 1U, 1000U);

    g_screw_home_watch.pos_counts = pos_counts;
    g_screw_pos_watch.active = 1U;
    g_screw_pos_watch.not_homed = 0U;
    g_screw_pos_watch.target_um = target_um;
    g_screw_pos_watch.pos_um = pos_um;
    g_screw_pos_watch.error_um = error_um;
    g_screw_pos_watch.target_counts = um_to_counts(target_um);
    g_screw_pos_watch.pos_counts = pos_counts;
    g_screw_pos_watch.limited = (uint8_t)(target_um != g_screw_pos_cmd.target_um);

    if ((error_um <= (int32_t)deadband_um) && (error_um >= -(int32_t)deadband_um))
    {
        speed_rpm = 0;
        g_screw_pos_watch.at_target = 1U;
    }
    else
    {
        speed_rpm = (error_um * (int32_t)kp_rpm_per_mm) / 1000L;
        speed_rpm =
            clamp_s32(speed_rpm, (int32_t)-max_speed_rpm, (int32_t)max_speed_rpm);
        g_screw_pos_watch.at_target = 0U;
    }

    apply_common_speed_command(iq_limit);
    g_motor_cmd.enable = 1U;
    g_motor_cmd.speed_ref_rpm = (int16_t)speed_rpm;
    g_screw_pos_watch.speed_ref_rpm = (int16_t)speed_rpm;
    return 1U;
}

static void jog_update(void)
{
    const uint32_t now_ms = app_millis();
    const uint32_t elapsed_ms = now_ms - s_jog.phase_start_ms;

    if (g_screw_jog_cmd.stop != 0U)
    {
        s_jog.phase = SCREW_JOG_PHASE_IDLE;
        stop_speed_command(0U);
        g_screw_jog_watch.busy = 0U;
        return;
    }

    if ((g_motor_watch.state == 4U) || (g_motor_watch.fault_reason != 0U))
    {
        s_jog.phase = SCREW_JOG_PHASE_FAULT;
        g_screw_jog_watch.fault_seen = 1U;
    }

    switch (s_jog.phase)
    {
        case SCREW_JOG_PHASE_IDLE:
            stop_speed_command(0U);
            g_screw_jog_watch.busy = 0U;
            g_screw_jog_watch.phase = s_jog.phase;
            g_screw_jog_watch.elapsed_ms = 0U;
            g_screw_jog_watch.remaining_ms = 0U;
            if (g_screw_jog_cmd.start_seq != s_jog.last_start_seq)
            {
                s_jog.last_start_seq = g_screw_jog_cmd.start_seq;
                s_jog.active_speed_rpm =
                    clamp_s16(g_screw_jog_cmd.speed_rpm,
                              (int16_t)-SCREW_JOG_MAX_SPEED_RPM,
                              SCREW_JOG_MAX_SPEED_RPM);
                s_jog.active_base_speed_rpm = s_jog.active_speed_rpm;
                s_jog.active_run_ms =
                    clamp_u16(g_screw_jog_cmd.run_ms, 1U, SCREW_JOG_MAX_RUN_MS);
                s_jog.active_iq_limit =
                    clamp_s16(g_screw_jog_cmd.iq_limit, 1, SCREW_JOG_MAX_IQ_LIMIT);
                s_jog.active_repeat_pairs =
                    clamp_u16(g_screw_jog_cmd.repeat_pairs, 0U,
                              SCREW_JOG_MAX_REPEAT_PAIRS);
                s_jog.active_leg_index = 0U;
                s_jog.phase_start_ms = now_ms;
                s_jog.phase = SCREW_JOG_PHASE_RUNNING;
                g_screw_jog_watch.active_seq = s_jog.last_start_seq;
                g_screw_jog_watch.active_speed_rpm = s_jog.active_speed_rpm;
                g_screw_jog_watch.active_run_ms = s_jog.active_run_ms;
                g_screw_jog_watch.active_repeat_pairs = s_jog.active_repeat_pairs;
                g_screw_jog_watch.completed_pairs = 0U;
                g_screw_jog_watch.leg_index = 0U;
            }
            break;

        case SCREW_JOG_PHASE_RUNNING:
            s_jog.active_speed_rpm = ((s_jog.active_leg_index & 1U) == 0U)
                                         ? s_jog.active_base_speed_rpm
                                         : (int16_t)-s_jog.active_base_speed_rpm;
            apply_common_speed_command(s_jog.active_iq_limit);
            g_motor_cmd.enable = 1U;
            g_motor_cmd.speed_ref_rpm = s_jog.active_speed_rpm;
            g_screw_jog_watch.busy = 1U;
            g_screw_jog_watch.phase = s_jog.phase;
            g_screw_jog_watch.active_speed_rpm = s_jog.active_speed_rpm;
            g_screw_jog_watch.leg_index = s_jog.active_leg_index;
            g_screw_jog_watch.completed_pairs = s_jog.active_leg_index / 2U;
            g_screw_jog_watch.elapsed_ms =
                (elapsed_ms > 65535U) ? 65535U : (uint16_t)elapsed_ms;
            g_screw_jog_watch.remaining_ms =
                (elapsed_ms >= s_jog.active_run_ms)
                    ? 0U
                    : (uint16_t)(s_jog.active_run_ms - elapsed_ms);
            if (elapsed_ms >= s_jog.active_run_ms)
            {
                if (s_jog.active_repeat_pairs == 0U)
                {
                    s_jog.phase = SCREW_JOG_PHASE_IDLE;
                    stop_speed_command(0U);
                }
                else
                {
                    s_jog.active_leg_index++;
                    if (s_jog.active_leg_index >=
                        (uint16_t)(s_jog.active_repeat_pairs * 2U))
                    {
                        s_jog.phase = SCREW_JOG_PHASE_IDLE;
                        stop_speed_command(0U);
                    }
                    else
                    {
                        s_jog.phase = SCREW_JOG_PHASE_PAUSE;
                        s_jog.phase_start_ms = now_ms;
                        stop_speed_command(1U);
                    }
                }
            }
            break;

        case SCREW_JOG_PHASE_PAUSE:
            stop_speed_command(1U);
            g_screw_jog_watch.busy = 1U;
            g_screw_jog_watch.phase = s_jog.phase;
            g_screw_jog_watch.active_speed_rpm = 0;
            g_screw_jog_watch.leg_index = s_jog.active_leg_index;
            g_screw_jog_watch.completed_pairs = s_jog.active_leg_index / 2U;
            g_screw_jog_watch.elapsed_ms =
                (elapsed_ms > 65535U) ? 65535U : (uint16_t)elapsed_ms;
            g_screw_jog_watch.remaining_ms =
                (elapsed_ms >= SCREW_JOG_PAUSE_MS)
                    ? 0U
                    : (uint16_t)(SCREW_JOG_PAUSE_MS - elapsed_ms);
            if (elapsed_ms >= SCREW_JOG_PAUSE_MS)
            {
                s_jog.phase = SCREW_JOG_PHASE_RUNNING;
                s_jog.phase_start_ms = now_ms;
            }
            break;

        case SCREW_JOG_PHASE_FAULT:
        default:
            stop_speed_command(0U);
            g_screw_jog_watch.busy = 0U;
            g_screw_jog_watch.phase = s_jog.phase;
            break;
    }
}

static int16_t abs_s16(int16_t value)
{
    if (value < 0)
    {
        return (int16_t)-value;
    }
    return value;
}

static int16_t clamp_s16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static uint16_t clamp_u16(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static int32_t clamp_s32(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}

static int32_t counts_to_um(int32_t counts)
{
    return (int32_t)(((int64_t)counts * 1000LL) / SCREW_COUNTS_PER_MM);
}

static int32_t um_to_counts(int32_t um)
{
    return (int32_t)(((int64_t)um * SCREW_COUNTS_PER_MM) / 1000LL);
}

static void apply_common_speed_command(int16_t iq_limit)
{
    g_motor_cmd.control_mode = SCREW_MC_MODE_SPEED;
    g_motor_cmd.id_ref = 0;
    g_motor_cmd.iq_ref = 0;
    g_motor_cmd.speed_ref = 0;
    g_motor_cmd.iq_limit = iq_limit;
    g_motor_cmd.elec_zero_trim = SCREW_ELEC_ZERO_TRIM;
}

static void stop_speed_command(uint8_t keep_enabled)
{
    g_motor_cmd.enable = keep_enabled;
    g_motor_cmd.control_mode = SCREW_MC_MODE_SPEED;
    g_motor_cmd.speed_ref = 0;
    g_motor_cmd.speed_ref_rpm = 0;
    g_motor_cmd.iq_ref = 0;
}

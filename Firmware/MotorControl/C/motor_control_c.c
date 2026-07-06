#include "motor_control_c.h"

#include "Config.h"
#include "foc_bsp.h"
#include "foc_curr.h"
#include "foc_math.h"
#include "foc_pwm.h"

#define MC_STATE_IDLE 0U
#define MC_STATE_CLOSED_LOOP 3U
#define MC_STATE_FAULT 4U

#define MC_MODE_OFF 0U
#define MC_MODE_CURRENT 1U
#define MC_MODE_SPEED 2U
#define MC_MODE_VF_OPEN_LOOP 3U
#define MC_MODE_ALIGN_LOCK 4U
#define MC_MODE_ENCODER_VOLTAGE 5U

#define MC_FAULT_NONE 0U
#define MC_FAULT_UNSUPPORTED_MODE 1U
#define MC_FAULT_CURRENT 2U
#define MC_FAULT_OPEN_LOOP_TIMEOUT 3U
#define MC_FAULT_ENCODER 4U

#define MC_ALIGN_STAGE_REV_FAST 1U
#define MC_ALIGN_STAGE_FWD_FAST 2U
#define MC_ALIGN_STAGE_FWD_SAMPLE 3U
#define MC_ALIGN_STAGE_REV_SAMPLE 4U
#define MC_ALIGN_STAGE_DONE 5U

#define MC_CURRENT_HZ (PWM_FREQ_HZ / CTRL_FAST_LOOP_DIV)
#define MC_SPEED_SAMPLE_DIV (MC_CURRENT_HZ / CTRL_SPD_EST_HZ)
#define MC_SPEED_COUNTS_PER_REV ((int32_t)MOT_SENSOR_CPR * (int32_t)MOT_SENSOR_POLE_PAIRS)

#if (CTRL_SPD_FB_SOURCE != CTRL_SPD_FB_SOURCE_DIFF) && \
    (CTRL_SPD_FB_SOURCE != CTRL_SPD_FB_SOURCE_MA600)
#error "Unsupported CTRL_SPD_FB_SOURCE"
#endif

typedef struct
{
    uint8_t state;
    uint8_t fault;
    uint8_t enabled;
    uint8_t mode;
    uint8_t pwm_output;
    uint32_t command_apply_count;
    uint32_t slow_loop_count;
    uint32_t fast_loop_count;
    uint32_t open_loop_ticks;
    uint32_t open_loop_timeout_ticks;
    uint32_t align_ticks;
    uint16_t current_loop_div;
    uint16_t speed_sample_div;
    int32_t open_loop_theta_acc;
    uint16_t open_loop_theta;
    uint16_t encoder_raw;
    uint16_t encoder_elec;
    uint16_t encoder_prev_raw;
    int16_t encoder_delta;
    int32_t encoder_pos;
    int32_t speed_fb;
    int32_t speed_fb_diff;
    int32_t speed_fb_ma600;
    int16_t ma600_speed_raw;
    int16_t speed_err_rpm;
    int16_t speed_iq_ref;
    uint8_t align_done;
    uint16_t align_raw;
    int16_t align_zero_trim;
    uint16_t align_encoder_elec;
    uint8_t align_stage;
    uint16_t align_halfcycles;
    int16_t align_theta_prev;
    int16_t align_first_delta;
    int16_t align_pull_delta;
    uint16_t align_sample_count;
    int32_t align_delta_sum;
    uint8_t encoder_age;
    uint8_t encoder_ok;
    uint8_t encoder_initialized;
    FocPi_t speed_pi;
    FocPi_t current_pi_d;
    FocPi_t current_pi_q;
    MotorControlCommand_t command;
    FocPhaseCurrent_t current;
    FocDq_t current_dq;
    int16_t id_ref_active;
    int16_t iq_ref_active;
    FocAlphaBeta_t voltage_ab;
    FocDq_t voltage_dq;
    uint16_t voltage_theta;
    FocDuty_t duty;
    uint8_t voltage_limited;
    MotorControlCheck_t check;
} MotorControlCState;

static MotorControlCState s_mc;

static void copy_command(const volatile MotorControlCommand_t* src, MotorControlCommand_t* dst);
static int16_t clamp_ref(int16_t value, int16_t limit);
static int16_t abs_limit(int16_t value, int16_t limit);
static int32_t clamp_s32_local(int32_t value, int32_t limit);
static int32_t rpm_to_speed_counts(int16_t rpm);
static int16_t speed_counts_to_rpm(int32_t speed);
#if (CTRL_SPD_FB_SOURCE == CTRL_SPD_FB_SOURCE_MA600)
static int32_t ma600_speed_to_counts(int16_t speed_raw);
static int32_t rpm_to_speed_counts_s32(int32_t rpm);
#endif
static int16_t slew_s16(int16_t current, int16_t target, int16_t step);
static int16_t align_trim_from_raw(uint16_t raw, uint16_t target_theta);
static uint8_t current_ok(void);
static uint16_t electrical_from_raw(uint16_t raw, int16_t trim);
static int16_t current_voltage_limit(void);
static void reset_encoder_state(void);
static void reset_current_loop(void);
static void reset_speed_loop(void);
static void reset_align_state(void);
static uint8_t update_encoder_angle(void);
static uint8_t update_encoder_speed(void);
static void update_speed_loop(void);
static void apply_voltage_vector(int16_t vd, int16_t vq, uint16_t theta);
static void enter_safe_state(void);
static void set_align_stage(uint8_t stage);
static uint16_t update_align_theta(int32_t speed);
static void count_align_halfcycle(uint16_t theta);
static void sample_align_trim(uint16_t theta);
static void finish_align_scan(void);
static void run_align_fast_loop(void);
static void run_vf_fast_loop(void);
static void run_encoder_voltage_fast_loop(void);
static void run_current_fast_loop(uint8_t speed_mode);
static uint16_t update_open_loop_theta(void);
static void fill_watch(MotorControlWatch_t* out);
static void copy_watch_to_volatile(volatile MotorControlWatch_t* dst,
                                   const MotorControlWatch_t* src);

void MotorControl_Init(void)
{
    s_mc.state = MC_STATE_IDLE;
    s_mc.fault = MC_FAULT_NONE;
    s_mc.enabled = 0U;
    s_mc.mode = MC_MODE_OFF;
    s_mc.pwm_output = 0U;
    s_mc.command_apply_count = 0U;
    s_mc.slow_loop_count = 0U;
    s_mc.fast_loop_count = 0U;
    s_mc.open_loop_ticks = 0U;
    s_mc.open_loop_timeout_ticks = (uint32_t)OL_TIMEOUT_MS * 2U;
    s_mc.align_ticks = 0U;
    s_mc.current_loop_div = 0U;
    s_mc.speed_sample_div = 0U;
    s_mc.open_loop_theta_acc = 0;
    s_mc.open_loop_theta = 0U;
    reset_encoder_state();
    foc_pi_init(&s_mc.speed_pi, CTRL_SPD_KP, CTRL_SPD_KI, -CTRL_SPD_IQ_LIMIT,
                CTRL_SPD_IQ_LIMIT, CTRL_SPD_ERR_SHIFT);
    foc_pi_init(&s_mc.current_pi_d, CTRL_CUR_KP, CTRL_CUR_KI, -CTRL_CUR_V_LIMIT,
                CTRL_CUR_V_LIMIT, CTRL_CUR_PI_SHIFT);
    foc_pi_init(&s_mc.current_pi_q, CTRL_CUR_KP, CTRL_CUR_KI, -CTRL_CUR_V_LIMIT,
                CTRL_CUR_V_LIMIT, CTRL_CUR_PI_SHIFT);
    s_mc.current = (FocPhaseCurrent_t){0, 0, 0};
    s_mc.current_dq = (FocDq_t){0, 0};
    s_mc.id_ref_active = 0;
    s_mc.iq_ref_active = 0;
    s_mc.voltage_ab = (FocAlphaBeta_t){0, 0};
    s_mc.voltage_dq = (FocDq_t){0, 0};
    s_mc.voltage_theta = 0U;
    s_mc.duty = (FocDuty_t){PWM_DUTY_50, PWM_DUTY_50, PWM_DUTY_50};
    s_mc.voltage_limited = 0U;
    reset_align_state();
    s_mc.check = (MotorControlCheck_t){0U, 1U, 1U, 0U};
    pwm_off();
}

void MotorControl_ApplyCommand(const volatile MotorControlCommand_t* command)
{
    if (command == 0)
    {
        return;
    }

    uint8_t next_mode;

    s_mc.command_apply_count++;
    copy_command(command, &s_mc.command);
    s_mc.enabled = (uint8_t)(s_mc.command.enable != 0U);
    next_mode = s_mc.enabled ? s_mc.command.control_mode : MC_MODE_OFF;
    if (next_mode != s_mc.mode)
    {
        reset_speed_loop();
        reset_current_loop();
        if (next_mode == MC_MODE_ALIGN_LOCK)
        {
            reset_align_state();
        }
    }
    s_mc.mode = next_mode;
    s_mc.command.iq_limit = abs_limit(s_mc.command.iq_limit, CTRL_CUR_REF_LIMIT);
    s_mc.command.current_kp = foc_clamp_s16(s_mc.command.current_kp, 0, 32767);
    s_mc.command.current_ki = foc_clamp_s16(s_mc.command.current_ki, 0, 32767);
    s_mc.command.speed_kp = foc_clamp_s16(s_mc.command.speed_kp, 0, 32767);
    s_mc.command.speed_ki = foc_clamp_s16(s_mc.command.speed_ki, 0, 32767);
    s_mc.command.current_v_limit = abs_limit(s_mc.command.current_v_limit,
                                             (int16_t)PWM_SVPWM_V_LIMIT);
    if (s_mc.mode == MC_MODE_ENCODER_VOLTAGE)
    {
        s_mc.command.id_ref = clamp_ref(s_mc.command.id_ref, s_mc.command.current_v_limit);
        s_mc.command.iq_ref = clamp_ref(s_mc.command.iq_ref, s_mc.command.current_v_limit);
    }
    else
    {
        s_mc.command.id_ref = clamp_ref(s_mc.command.id_ref, CTRL_CUR_REF_LIMIT);
        s_mc.command.iq_ref = clamp_ref(s_mc.command.iq_ref, s_mc.command.iq_limit);
    }
    s_mc.command.open_loop_speed_ref =
        clamp_s32_local(s_mc.command.open_loop_speed_ref, CTRL_SPD_REF_LIMIT);
    if (s_mc.command.speed_ref_rpm != 0)
    {
        s_mc.command.speed_ref = rpm_to_speed_counts(s_mc.command.speed_ref_rpm);
    }
    s_mc.command.speed_ref = clamp_s32_local(s_mc.command.speed_ref, CTRL_SPD_REF_LIMIT);
    s_mc.command.vf_voltage = clamp_ref(s_mc.command.vf_voltage, CTRL_CUR_V_LIMIT);
    s_mc.open_loop_timeout_ticks = (uint32_t)s_mc.command.open_loop_timeout_ms * 2U;
    if ((s_mc.enabled != 0U) && (s_mc.mode == MC_MODE_VF_OPEN_LOOP))
    {
        curr_set_vf_voltage(s_mc.command.vf_voltage);
    }
    else
    {
        curr_set_vf_voltage(0);
    }

    if (s_mc.enabled == 0U)
    {
        if ((s_mc.pwm_output != 0U) || (s_mc.state != MC_STATE_IDLE))
        {
            enter_safe_state();
        }
        s_mc.state = MC_STATE_IDLE;
        s_mc.fault = MC_FAULT_NONE;
    }
}

void MotorControl_RunSlowLoop(void)
{
    s_mc.current.u = curr_u();
    s_mc.current.v = curr_v();
    s_mc.current.w = curr_w();
    s_mc.check.current_ok = current_ok();
    s_mc.check.pwm_off_safe = (pwm_is_off_safe() != 0U) ? 1U : 0U;
    s_mc.check.ma600_ok = 0U;
    s_mc.check.ready_closed_loop = 0U;

    if (s_mc.enabled == 0U)
    {
        s_mc.slow_loop_count++;
        return;
    }

    if ((s_mc.mode == MC_MODE_VF_OPEN_LOOP) || (s_mc.mode == MC_MODE_CURRENT) ||
        (s_mc.mode == MC_MODE_SPEED) || (s_mc.mode == MC_MODE_ALIGN_LOCK) ||
        (s_mc.mode == MC_MODE_ENCODER_VOLTAGE))
    {
        s_mc.check.ma600_ok =
            ((s_mc.mode == MC_MODE_CURRENT) || (s_mc.mode == MC_MODE_SPEED) ||
             (s_mc.mode == MC_MODE_ENCODER_VOLTAGE))
                ? (uint8_t)((s_mc.encoder_ok != 0U) || (s_mc.encoder_initialized == 0U))
                : 1U;
        s_mc.check.ready_closed_loop =
            (uint8_t)((s_mc.check.current_ok != 0U) &&
                      ((s_mc.mode == MC_MODE_VF_OPEN_LOOP) ||
                       (s_mc.mode == MC_MODE_ALIGN_LOCK) || (s_mc.encoder_ok != 0U) ||
                       (s_mc.encoder_initialized == 0U)));

        if (s_mc.check.ready_closed_loop != 0U)
        {
            if (s_mc.state != MC_STATE_CLOSED_LOOP)
            {
                s_mc.open_loop_ticks = 0U;
                s_mc.open_loop_theta = 0U;
                s_mc.open_loop_theta_acc = 0;
                if (s_mc.mode != MC_MODE_VF_OPEN_LOOP)
                {
                    reset_speed_loop();
                }
                reset_current_loop();
            }
            s_mc.state = MC_STATE_CLOSED_LOOP;
            s_mc.fault = MC_FAULT_NONE;
        }
        else
        {
            s_mc.state = MC_STATE_FAULT;
            s_mc.fault = MC_FAULT_CURRENT;
            enter_safe_state();
        }
    }
    else
    {
        s_mc.state = MC_STATE_FAULT;
        s_mc.fault = MC_FAULT_UNSUPPORTED_MODE;
        enter_safe_state();
    }

    s_mc.slow_loop_count++;
}

uint8_t MotorControl_FastLoopFromAdcIrq(void)
{
    const uint8_t sample_ready = bsp_adc_irq();
    if (sample_ready == 0U)
    {
        return 0U;
    }

    if ((s_mc.enabled != 0U) && (s_mc.state == MC_STATE_CLOSED_LOOP))
    {
        if (s_mc.mode == MC_MODE_VF_OPEN_LOOP)
        {
            run_vf_fast_loop();
        }
        else if (s_mc.mode == MC_MODE_ALIGN_LOCK)
        {
            run_align_fast_loop();
        }
        else if (s_mc.mode == MC_MODE_ENCODER_VOLTAGE)
        {
            run_encoder_voltage_fast_loop();
        }
        else if (s_mc.mode == MC_MODE_SPEED)
        {
            run_current_fast_loop(1U);
        }
        else if (s_mc.mode == MC_MODE_CURRENT)
        {
            run_current_fast_loop(0U);
        }
    }
    return 1U;
}

void MotorControl_GetWatch(MotorControlWatch_t* out)
{
    if (out == 0)
    {
        return;
    }
    fill_watch(out);
}

void MotorControl_UpdateWatch(volatile MotorControlWatch_t* out)
{
    MotorControlWatch_t snapshot;

    if (out == 0)
    {
        return;
    }

    fill_watch(&snapshot);
    copy_watch_to_volatile(out, &snapshot);
}

static void copy_command(const volatile MotorControlCommand_t* src, MotorControlCommand_t* dst)
{
    dst->enable = src->enable;
    dst->control_mode = src->control_mode;
    dst->id_ref = src->id_ref;
    dst->iq_ref = src->iq_ref;
    dst->speed_ref = src->speed_ref;
    dst->speed_ref_rpm = src->speed_ref_rpm;
    dst->iq_limit = src->iq_limit;
    dst->current_kp = src->current_kp;
    dst->current_ki = src->current_ki;
    dst->speed_kp = src->speed_kp;
    dst->speed_ki = src->speed_ki;
    dst->current_v_limit = src->current_v_limit;
    dst->open_loop_speed_ref = src->open_loop_speed_ref;
    dst->vf_voltage = src->vf_voltage;
    dst->if_id_ref = src->if_id_ref;
    dst->if_iq_ref = src->if_iq_ref;
    dst->open_loop_timeout_ms = src->open_loop_timeout_ms;
    dst->elec_zero_trim = src->elec_zero_trim;
    dst->voltage_theta_offset = src->voltage_theta_offset;
}

static int16_t clamp_ref(int16_t value, int16_t limit)
{
    return foc_clamp_s16(value, (int16_t)-limit, limit);
}

static int16_t abs_limit(int16_t value, int16_t limit)
{
    if (value < 0)
    {
        value = (int16_t)-value;
    }
    return foc_clamp_s16(value, 0, limit);
}

static int32_t clamp_s32_local(int32_t value, int32_t limit)
{
    return foc_clamp_s32(value, -limit, limit);
}

static int32_t rpm_to_speed_counts(int16_t rpm)
{
    return ((int32_t)rpm * MC_SPEED_COUNTS_PER_REV) / 60L;
}

#if (CTRL_SPD_FB_SOURCE == CTRL_SPD_FB_SOURCE_MA600)
static int32_t rpm_to_speed_counts_s32(int32_t rpm)
{
    return (rpm * MC_SPEED_COUNTS_PER_REV) / 60L;
}

static int32_t ma600_speed_to_counts(int16_t speed_raw)
{
    return (int32_t)speed_raw * (int32_t)CTRL_SPD_MA600_COUNTS_PER_SEC_PER_LSB *
           (int32_t)MOT_SENSOR_DIR * (int32_t)CTRL_SPD_MA600_SIGN;
}
#endif

static int16_t speed_counts_to_rpm(int32_t speed)
{
    int32_t rpm = (speed * 60L) / MC_SPEED_COUNTS_PER_REV;
    return (int16_t)foc_clamp_s32(rpm, -32768, 32767);
}

static int16_t slew_s16(int16_t current, int16_t target, int16_t step)
{
    int32_t delta;

    if (step <= 0)
    {
        return target;
    }

    delta = (int32_t)target - (int32_t)current;
    if (delta > step)
    {
        return (int16_t)(current + step);
    }
    if (delta < -step)
    {
        return (int16_t)(current - step);
    }
    return target;
}

static int16_t align_trim_from_raw(uint16_t raw, uint16_t target_theta)
{
    int32_t trim;

#if MOT_SENSOR_DIR > 0
    trim = (int32_t)target_theta - (int32_t)MOT_ELEC_ZERO -
           (int32_t)raw * (int32_t)MOT_SENSOR_ELEC;
#else
    trim = (int32_t)target_theta - (int32_t)MOT_ELEC_ZERO +
           (int32_t)raw * (int32_t)MOT_SENSOR_ELEC;
#endif
    return (int16_t)trim;
}

static uint8_t current_ok(void)
{
    uint8_t ok = 1U;
#if (MOT_CHECK_CURR_CNT_LIMIT < 32767)
    ok = (uint8_t)(ok && (s_mc.current.u >= -MOT_CHECK_CURR_CNT_LIMIT) &&
                  (s_mc.current.u <= MOT_CHECK_CURR_CNT_LIMIT));
    ok = (uint8_t)(ok && (s_mc.current.v >= -MOT_CHECK_CURR_CNT_LIMIT) &&
                  (s_mc.current.v <= MOT_CHECK_CURR_CNT_LIMIT));
    ok = (uint8_t)(ok && (s_mc.current.w >= -MOT_CHECK_CURR_CNT_LIMIT) &&
                  (s_mc.current.w <= MOT_CHECK_CURR_CNT_LIMIT));
#endif
#if (MOT_CHECK_SUM_CNT_LIMIT < 32767)
    {
        const int16_t sum = (int16_t)(s_mc.current.u + s_mc.current.v + s_mc.current.w);
        ok = (uint8_t)(ok && (sum >= -MOT_CHECK_SUM_CNT_LIMIT) &&
                      (sum <= MOT_CHECK_SUM_CNT_LIMIT));
    }
#endif
    return ok;
}

static uint16_t electrical_from_raw(uint16_t raw, int16_t trim)
{
    const int32_t zero = (int32_t)MOT_ELEC_ZERO + (int32_t)trim;
#if MOT_SENSOR_DIR > 0
    return (uint16_t)(zero + (int32_t)raw * (int32_t)MOT_SENSOR_ELEC);
#else
    return (uint16_t)(zero - (int32_t)raw * (int32_t)MOT_SENSOR_ELEC);
#endif
}

static int16_t current_voltage_limit(void)
{
    int16_t limit = s_mc.command.current_v_limit;

    if (limit <= 0)
    {
        limit = CTRL_CUR_V_LIMIT;
    }
    return limit;
}

static void reset_encoder_state(void)
{
    s_mc.encoder_raw = 0U;
    s_mc.encoder_elec = 0U;
    s_mc.encoder_prev_raw = 0U;
    s_mc.encoder_delta = 0;
    s_mc.encoder_pos = 0;
    s_mc.speed_fb = 0;
    s_mc.speed_fb_diff = 0;
    s_mc.speed_fb_ma600 = 0;
    s_mc.ma600_speed_raw = 0;
    s_mc.speed_err_rpm = 0;
    s_mc.speed_iq_ref = 0;
    s_mc.encoder_age = 255U;
    s_mc.encoder_ok = 0U;
    s_mc.encoder_initialized = 0U;
}

static void reset_current_loop(void)
{
    foc_pi_reset(&s_mc.current_pi_d);
    foc_pi_reset(&s_mc.current_pi_q);
    s_mc.current_loop_div = 0U;
    s_mc.current_dq = (FocDq_t){0, 0};
    s_mc.id_ref_active = 0;
    s_mc.iq_ref_active = 0;
}

static void reset_speed_loop(void)
{
    reset_encoder_state();
    foc_pi_reset(&s_mc.speed_pi);
    s_mc.speed_sample_div = 0U;
}

static void reset_align_state(void)
{
    s_mc.align_ticks = 0U;
    s_mc.align_done = 0U;
    s_mc.align_raw = 0U;
    s_mc.align_zero_trim = 0;
    s_mc.align_encoder_elec = 0U;
    s_mc.align_stage = MC_ALIGN_STAGE_REV_FAST;
    s_mc.align_halfcycles = 0U;
    s_mc.align_theta_prev = 0;
    s_mc.align_first_delta = 0;
    s_mc.align_pull_delta = 0;
    s_mc.align_sample_count = 0U;
    s_mc.align_delta_sum = 0;
    s_mc.open_loop_ticks = 0U;
    s_mc.open_loop_theta = 0U;
    s_mc.open_loop_theta_acc = 0;
}

static uint8_t update_encoder_angle(void)
{
    uint8_t ok;
    uint16_t raw;

    ok = bsp_update_angle_fast();
    raw = bsp_angle_raw();
    s_mc.encoder_ok = (uint8_t)((ok != 0U) && (bsp_angle_ok() != 0U));
    s_mc.encoder_age = bsp_angle_age();

    if (s_mc.encoder_ok == 0U)
    {
        if (s_mc.encoder_age < 255U)
        {
            s_mc.encoder_age++;
        }
        return 0U;
    }

    s_mc.encoder_raw = raw;
    s_mc.encoder_elec = electrical_from_raw(raw, s_mc.command.elec_zero_trim);

    if (s_mc.encoder_initialized == 0U)
    {
        s_mc.encoder_prev_raw = raw;
        s_mc.encoder_delta = 0;
        s_mc.encoder_initialized = 1U;
    }
    return 1U;
}

static uint8_t update_encoder_speed(void)
{
    uint16_t raw;
    int16_t delta;
    int32_t speed_diff;

#if (CTRL_SPD_FB_SOURCE == CTRL_SPD_FB_SOURCE_MA600)
    int32_t speed_ma600;

    if (bsp_update_angle_speed_fast() == 0U)
    {
        s_mc.encoder_ok = 0U;
        s_mc.encoder_age = bsp_angle_age();
        return 0U;
    }

    raw = s_mc.encoder_raw;
    s_mc.encoder_ok = (uint8_t)(bsp_angle_ok() != 0U);
    s_mc.encoder_age = bsp_angle_age();
    s_mc.encoder_raw = bsp_angle_raw();
    raw = s_mc.encoder_raw;
    s_mc.encoder_elec = electrical_from_raw(raw, s_mc.command.elec_zero_trim);
#else
    if (update_encoder_angle() == 0U)
    {
        return 0U;
    }
    raw = s_mc.encoder_raw;
#endif

    if (s_mc.encoder_initialized == 0U)
    {
        s_mc.encoder_prev_raw = raw;
        s_mc.encoder_delta = 0;
        s_mc.encoder_initialized = 1U;
    }

    delta = (int16_t)(raw - s_mc.encoder_prev_raw);
    s_mc.encoder_prev_raw = raw;
    s_mc.encoder_delta = delta;
    s_mc.encoder_pos += delta;

    if ((delta > -CTRL_SPD_POS_DEADBAND) && (delta < CTRL_SPD_POS_DEADBAND))
    {
        delta = 0;
    }

    speed_diff = (int32_t)delta * (int32_t)CTRL_SPD_EST_HZ * (int32_t)MOT_SENSOR_DIR;
    s_mc.speed_fb_diff += (speed_diff - s_mc.speed_fb_diff) >> CTRL_SPD_FILTER_SHIFT;

#if (CTRL_SPD_FB_SOURCE == CTRL_SPD_FB_SOURCE_MA600)
    s_mc.ma600_speed_raw = bsp_angle_speed_raw();
    speed_ma600 = ma600_speed_to_counts(s_mc.ma600_speed_raw);
    if ((speed_ma600 - s_mc.speed_fb_ma600 <=
         rpm_to_speed_counts_s32(CTRL_SPD_MA600_SPIKE_RPM)) &&
        (speed_ma600 - s_mc.speed_fb_ma600 >=
         -rpm_to_speed_counts_s32(CTRL_SPD_MA600_SPIKE_RPM)))
    {
        s_mc.speed_fb_ma600 +=
            (speed_ma600 - s_mc.speed_fb_ma600) >> CTRL_SPD_MA600_FILTER_SHIFT;
    }
#else
    s_mc.ma600_speed_raw = 0;
    s_mc.speed_fb_ma600 = 0;
#endif

#if (CTRL_SPD_FB_SOURCE == CTRL_SPD_FB_SOURCE_MA600)
    s_mc.speed_fb = s_mc.speed_fb_ma600;
#else
    s_mc.speed_fb = s_mc.speed_fb_diff;
#endif

    if ((s_mc.speed_fb > -CTRL_SPD_ZERO_SNAP) && (s_mc.speed_fb < CTRL_SPD_ZERO_SNAP))
    {
        s_mc.speed_fb = 0;
    }
    return 1U;
}

static void update_speed_loop(void)
{
    const int16_t ref_rpm = speed_counts_to_rpm(s_mc.command.speed_ref);
    const int16_t fb_rpm = speed_counts_to_rpm(s_mc.speed_fb);

    s_mc.speed_err_rpm = (int16_t)foc_clamp_s32((int32_t)ref_rpm - (int32_t)fb_rpm,
                                                -32768, 32767);

    if ((s_mc.command.speed_ref > -CTRL_SPD_CMD_DEADBAND) &&
        (s_mc.command.speed_ref < CTRL_SPD_CMD_DEADBAND))
    {
        foc_pi_reset(&s_mc.speed_pi);
        s_mc.speed_err_rpm = 0;
        s_mc.speed_iq_ref = 0;
        return;
    }

    foc_pi_set_gains(&s_mc.speed_pi, s_mc.command.speed_kp, s_mc.command.speed_ki,
                     (int16_t)-s_mc.command.iq_limit, s_mc.command.iq_limit,
                     CTRL_SPD_ERR_SHIFT);
    s_mc.speed_iq_ref = foc_pi_update(&s_mc.speed_pi, ref_rpm, fb_rpm);
}

static void enter_safe_state(void)
{
    pwm_off();
    s_mc.pwm_output = 0U;
    s_mc.voltage_dq = (FocDq_t){0, 0};
    s_mc.voltage_ab = (FocAlphaBeta_t){0, 0};
    s_mc.voltage_theta = 0U;
    s_mc.speed_iq_ref = 0;
    s_mc.speed_err_rpm = 0;
    s_mc.duty = (FocDuty_t){PWM_DUTY_50, PWM_DUTY_50, PWM_DUTY_50};
    foc_pi_reset(&s_mc.speed_pi);
}

static void apply_voltage_vector(int16_t vd, int16_t vq, uint16_t theta)
{
    const int16_t v_limit = current_voltage_limit();

    s_mc.voltage_theta = theta;
    s_mc.voltage_dq.d = vd;
    s_mc.voltage_dq.q = vq;
    s_mc.voltage_limited = foc_limit_dq(&s_mc.voltage_dq, v_limit);
    s_mc.voltage_ab = foc_inv_park(s_mc.voltage_dq, theta);
    s_mc.duty = foc_svpwm(s_mc.voltage_ab, PWM_PERIOD, PWM_DUTY_MIN, PWM_DUTY_MAX);
    pwm_set_duty(s_mc.duty.u, s_mc.duty.v, s_mc.duty.w);
    (void)pwm_enable(1U);
    s_mc.pwm_output = 1U;
}

static void set_align_stage(uint8_t stage)
{
    s_mc.align_stage = stage;
    s_mc.align_halfcycles = 0U;
    s_mc.align_theta_prev =
        (int16_t)((int32_t)s_mc.open_loop_theta * (int32_t)MOT_SENSOR_DIR);
}

static uint16_t update_align_theta(int32_t speed)
{
    int32_t step;

    step = speed * (int32_t)OL_SPEED_TO_THETA_STEP;
    if (step >= 0)
    {
        step += (1L << (OL_SPEED_TO_THETA_SHIFT - 1U));
    }
    else
    {
        step -= (1L << (OL_SPEED_TO_THETA_SHIFT - 1U));
    }
    step >>= OL_SPEED_TO_THETA_SHIFT;

    s_mc.open_loop_theta_acc += step;
    s_mc.open_loop_theta = (uint16_t)((uint32_t)s_mc.open_loop_theta_acc & 0xFFFFUL);
    s_mc.open_loop_ticks++;
    return (uint16_t)((int32_t)s_mc.open_loop_theta * (int32_t)MOT_SENSOR_DIR);
}

static void count_align_halfcycle(uint16_t theta)
{
    const int16_t theta_now = (int16_t)theta;

    if (((s_mc.align_theta_prev > 0) && (theta_now < 0)) ||
        ((s_mc.align_theta_prev < 0) && (theta_now > 0)))
    {
        if (s_mc.align_halfcycles < 0xFFFFU)
        {
            s_mc.align_halfcycles++;
        }
    }
    s_mc.align_theta_prev = theta_now;
}

static void sample_align_trim(uint16_t theta)
{
    const int16_t trim = align_trim_from_raw(s_mc.encoder_raw, theta);
    int16_t delta;

    s_mc.align_pull_delta = trim;
    if (s_mc.align_sample_count == 0U)
    {
        s_mc.align_first_delta = trim;
        s_mc.align_delta_sum = 0;
        s_mc.align_sample_count = 1U;
        return;
    }

    delta = (int16_t)((uint16_t)trim - (uint16_t)s_mc.align_first_delta);
    s_mc.align_delta_sum += (int32_t)delta;
    if (s_mc.align_sample_count < 0xFFFFU)
    {
        s_mc.align_sample_count++;
    }
}

static void finish_align_scan(void)
{
    int32_t average = s_mc.align_first_delta;

    if (s_mc.align_sample_count > 1U)
    {
        average += s_mc.align_delta_sum / (int32_t)(s_mc.align_sample_count - 1U);
    }

    s_mc.align_zero_trim = (int16_t)average;
    s_mc.align_done = 1U;
    set_align_stage(MC_ALIGN_STAGE_DONE);
}

static void run_align_fast_loop(void)
{
    int32_t speed = 0;
    uint16_t theta;

    s_mc.current.u = curr_u();
    s_mc.current.v = curr_v();
    s_mc.current.w = curr_w();
    if (current_ok() == 0U)
    {
        s_mc.state = MC_STATE_FAULT;
        s_mc.fault = MC_FAULT_CURRENT;
        enter_safe_state();
        return;
    }

    if (++s_mc.current_loop_div < CTRL_FAST_LOOP_DIV)
    {
        return;
    }
    s_mc.current_loop_div = 0U;

    if (s_mc.align_done != 0U)
    {
        apply_voltage_vector(0, 0, s_mc.voltage_theta);
        s_mc.fast_loop_count++;
        return;
    }

    if (s_mc.align_stage == MC_ALIGN_STAGE_REV_FAST)
    {
        speed = -MOT_ALIGN_SCAN_FAST_SPEED;
    }
    else if (s_mc.align_stage == MC_ALIGN_STAGE_FWD_FAST)
    {
        speed = MOT_ALIGN_SCAN_FAST_SPEED;
    }
    else if (s_mc.align_stage == MC_ALIGN_STAGE_FWD_SAMPLE)
    {
        speed = MOT_ALIGN_SCAN_SLOW_SPEED;
    }
    else if (s_mc.align_stage == MC_ALIGN_STAGE_REV_SAMPLE)
    {
        speed = -MOT_ALIGN_SCAN_SLOW_SPEED;
    }
    else
    {
        finish_align_scan();
        s_mc.fast_loop_count++;
        return;
    }

    theta = update_align_theta(speed);
    apply_voltage_vector(MOT_ALIGN_SCAN_VD, 0, theta);

    if (update_encoder_angle() != 0U)
    {
        s_mc.align_raw = s_mc.encoder_raw;
        s_mc.align_encoder_elec = s_mc.encoder_elec;
        if ((s_mc.align_stage == MC_ALIGN_STAGE_FWD_SAMPLE) ||
            (s_mc.align_stage == MC_ALIGN_STAGE_REV_SAMPLE))
        {
            sample_align_trim(theta);
        }
    }

    if (s_mc.align_ticks < 0xFFFFFFFFUL)
    {
        s_mc.align_ticks++;
    }

    count_align_halfcycle(theta);

    if ((s_mc.align_stage == MC_ALIGN_STAGE_REV_FAST) &&
        (s_mc.align_halfcycles >= MOT_ALIGN_SCAN_REV_FAST_HALFCYCLES))
    {
        set_align_stage(MC_ALIGN_STAGE_FWD_FAST);
    }
    else if ((s_mc.align_stage == MC_ALIGN_STAGE_FWD_FAST) &&
             (s_mc.align_halfcycles >= MOT_ALIGN_SCAN_FWD_FAST_HALFCYCLES))
    {
        set_align_stage(MC_ALIGN_STAGE_FWD_SAMPLE);
    }
    else if ((s_mc.align_stage == MC_ALIGN_STAGE_FWD_SAMPLE) &&
             (s_mc.align_halfcycles >= MOT_ALIGN_SCAN_SAMPLE_HALFCYCLES) &&
             (s_mc.align_sample_count >= MOT_ALIGN_SCAN_MIN_SAMPLES))
    {
        set_align_stage(MC_ALIGN_STAGE_REV_SAMPLE);
    }
    else if ((s_mc.align_stage == MC_ALIGN_STAGE_REV_SAMPLE) &&
             (s_mc.align_halfcycles >= MOT_ALIGN_SCAN_SAMPLE_HALFCYCLES) &&
             (s_mc.align_sample_count >= MOT_ALIGN_SCAN_MIN_SAMPLES))
    {
        finish_align_scan();
    }

    s_mc.fast_loop_count++;
}

static void run_vf_fast_loop(void)
{
    uint16_t theta;

    s_mc.current.u = curr_u();
    s_mc.current.v = curr_v();
    s_mc.current.w = curr_w();
    if (current_ok() == 0U)
    {
        s_mc.state = MC_STATE_FAULT;
        s_mc.fault = MC_FAULT_CURRENT;
        enter_safe_state();
        return;
    }

    if ((s_mc.open_loop_timeout_ticks > 0U) &&
        (s_mc.open_loop_ticks >= s_mc.open_loop_timeout_ticks))
    {
        s_mc.state = MC_STATE_FAULT;
        s_mc.fault = MC_FAULT_OPEN_LOOP_TIMEOUT;
        enter_safe_state();
        return;
    }

    theta = update_open_loop_theta();
    if (++s_mc.speed_sample_div >= MC_SPEED_SAMPLE_DIV)
    {
        s_mc.speed_sample_div = 0U;
        (void)update_encoder_speed();
    }
    apply_voltage_vector(s_mc.command.vf_voltage, 0, theta);
    s_mc.fast_loop_count++;
}

static void run_encoder_voltage_fast_loop(void)
{
    FocAlphaBeta_t current_ab;
    uint16_t voltage_theta;

    s_mc.current.u = curr_u();
    s_mc.current.v = curr_v();
    s_mc.current.w = curr_w();
    if (current_ok() == 0U)
    {
        s_mc.state = MC_STATE_FAULT;
        s_mc.fault = MC_FAULT_CURRENT;
        enter_safe_state();
        return;
    }

    if (++s_mc.current_loop_div < CTRL_FAST_LOOP_DIV)
    {
        return;
    }
    s_mc.current_loop_div = 0U;

    if (++s_mc.speed_sample_div >= MC_SPEED_SAMPLE_DIV)
    {
        s_mc.speed_sample_div = 0U;
        if (update_encoder_speed() == 0U)
        {
            s_mc.state = MC_STATE_FAULT;
            s_mc.fault = MC_FAULT_ENCODER;
            enter_safe_state();
            return;
        }
    }
    else if (update_encoder_angle() == 0U)
    {
        s_mc.state = MC_STATE_FAULT;
        s_mc.fault = MC_FAULT_ENCODER;
        enter_safe_state();
        return;
    }

    s_mc.id_ref_active = s_mc.command.id_ref;
    s_mc.iq_ref_active = s_mc.command.iq_ref;
    voltage_theta = (uint16_t)(s_mc.encoder_elec + (uint16_t)s_mc.command.voltage_theta_offset);
    current_ab = foc_clarke_3phase(s_mc.current);
    s_mc.current_dq = foc_park(current_ab, voltage_theta);
    apply_voltage_vector(s_mc.command.id_ref, s_mc.command.iq_ref, voltage_theta);
    s_mc.fast_loop_count++;
}

static void run_current_fast_loop(uint8_t speed_mode)
{
    FocAlphaBeta_t current_ab;
    uint16_t theta_used;
    int16_t v_limit;
    int16_t iq_ref;

    s_mc.current.u = curr_u();
    s_mc.current.v = curr_v();
    s_mc.current.w = curr_w();
    if (current_ok() == 0U)
    {
        s_mc.state = MC_STATE_FAULT;
        s_mc.fault = MC_FAULT_CURRENT;
        enter_safe_state();
        return;
    }

    if (++s_mc.current_loop_div < CTRL_FAST_LOOP_DIV)
    {
        return;
    }
    s_mc.current_loop_div = 0U;

    if (update_encoder_angle() == 0U)
    {
        s_mc.state = MC_STATE_FAULT;
        s_mc.fault = MC_FAULT_ENCODER;
        enter_safe_state();
        return;
    }

    if (speed_mode != 0U)
    {
        if (++s_mc.speed_sample_div >= MC_SPEED_SAMPLE_DIV)
        {
            s_mc.speed_sample_div = 0U;
            if (update_encoder_speed() == 0U)
            {
                s_mc.state = MC_STATE_FAULT;
                s_mc.fault = MC_FAULT_ENCODER;
                enter_safe_state();
                return;
            }
            update_speed_loop();
        }

        iq_ref = s_mc.speed_iq_ref;
    }
    else
    {
        iq_ref = s_mc.command.iq_ref;
    }

    s_mc.id_ref_active = slew_s16(s_mc.id_ref_active, s_mc.command.id_ref,
                                  CTRL_CUR_REF_RAMP_STEP);
    s_mc.iq_ref_active =
        slew_s16(s_mc.iq_ref_active,
                 foc_clamp_s16(iq_ref, (int16_t)-s_mc.command.iq_limit,
                               s_mc.command.iq_limit),
                 CTRL_CUR_REF_RAMP_STEP);

    theta_used = (uint16_t)(s_mc.encoder_elec + (uint16_t)s_mc.command.voltage_theta_offset);
    current_ab = foc_clarke_3phase(s_mc.current);
    s_mc.current_dq = foc_park(current_ab, theta_used);

    v_limit = current_voltage_limit();
    foc_pi_set_gains(&s_mc.current_pi_d, s_mc.command.current_kp, s_mc.command.current_ki,
                     (int16_t)-v_limit, v_limit, CTRL_CUR_PI_SHIFT);
    foc_pi_set_gains(&s_mc.current_pi_q, s_mc.command.current_kp, s_mc.command.current_ki,
                     (int16_t)-v_limit, v_limit, CTRL_CUR_PI_SHIFT);

    s_mc.voltage_dq.d = foc_pi_update(&s_mc.current_pi_d, s_mc.id_ref_active, s_mc.current_dq.d);
    s_mc.voltage_dq.q = foc_pi_update(&s_mc.current_pi_q, s_mc.iq_ref_active, s_mc.current_dq.q);
    s_mc.voltage_limited = foc_limit_dq(&s_mc.voltage_dq, v_limit);
    s_mc.voltage_theta = theta_used;
    s_mc.voltage_ab = foc_inv_park(s_mc.voltage_dq, theta_used);
    s_mc.duty = foc_svpwm(s_mc.voltage_ab, PWM_PERIOD, PWM_DUTY_MIN, PWM_DUTY_MAX);
    pwm_set_duty(s_mc.duty.u, s_mc.duty.v, s_mc.duty.w);
    (void)pwm_enable(1U);
    s_mc.pwm_output = 1U;
    s_mc.fast_loop_count++;
}

static uint16_t update_open_loop_theta(void)
{
    int32_t step = (s_mc.command.open_loop_speed_ref * (int32_t)OL_SPEED_TO_THETA_STEP +
                    (1L << (OL_SPEED_TO_THETA_SHIFT - 1U))) >>
                   OL_SPEED_TO_THETA_SHIFT;
    s_mc.open_loop_theta_acc += step;
    s_mc.open_loop_theta = (uint16_t)((uint32_t)s_mc.open_loop_theta_acc & 0xFFFFUL);
    s_mc.open_loop_ticks++;
    return (uint16_t)((int32_t)s_mc.open_loop_theta * (int32_t)MOT_SENSOR_DIR);
}

static void fill_watch(MotorControlWatch_t* out)
{
    volatile uint16_t duty_u = 0U;
    volatile uint16_t duty_v = 0U;
    volatile uint16_t duty_w = 0U;
    volatile uint8_t pwm_out = 0U;
    volatile uint8_t pwm_brake = 0U;

    pwm_snapshot(&duty_u, &duty_v, &duty_w, &pwm_out, &pwm_brake);
    (void)pwm_brake;

    out->state = s_mc.state;
    out->control_mode = s_mc.mode;
    out->fault_reason = s_mc.fault;
    out->enable = s_mc.enabled;
    out->slow_loop_count = s_mc.slow_loop_count;
    out->fast_loop_count = s_mc.fast_loop_count;
    out->adc_sample_count = curr_sync_count();
    out->encoder_raw = s_mc.encoder_raw;
    out->encoder_elec = s_mc.encoder_elec;
    out->encoder_delta = s_mc.encoder_delta;
    out->encoder_pos = s_mc.encoder_pos;
    out->encoder_age = s_mc.encoder_age;
    out->encoder_ok = s_mc.encoder_ok;
    out->align_done = s_mc.align_done;
    out->align_ticks = s_mc.align_ticks;
    out->align_theta = (uint16_t)MOT_ALIGN_THETA;
    out->align_raw = s_mc.align_raw;
    out->align_zero_trim = s_mc.align_zero_trim;
    out->align_encoder_elec = s_mc.align_encoder_elec;
    out->align_stage = s_mc.align_stage;
    out->align_pull_delta = s_mc.align_pull_delta;
    out->align_sample_count = s_mc.align_sample_count;
    out->align_delta_sum = s_mc.align_delta_sum;
    out->iu_cnt = curr_u();
    out->iv_cnt = curr_v();
    out->iw_cnt = curr_w();
    out->i_sum = curr_sum();
    out->id_ref = s_mc.id_ref_active;
    out->iq_ref = s_mc.iq_ref_active;
    out->speed_ref =
        (s_mc.mode == MC_MODE_SPEED) ? s_mc.command.speed_ref : s_mc.command.open_loop_speed_ref;
    out->speed_ref_rpm = (s_mc.mode == MC_MODE_SPEED) ? speed_counts_to_rpm(s_mc.command.speed_ref)
                                                       : 0;
    out->speed_fb = s_mc.speed_fb;
    out->speed_fb_rpm = speed_counts_to_rpm(s_mc.speed_fb);
    out->speed_fb_diff = s_mc.speed_fb_diff;
    out->speed_fb_diff_rpm = speed_counts_to_rpm(s_mc.speed_fb_diff);
    out->speed_fb_ma600 = s_mc.speed_fb_ma600;
    out->speed_fb_ma600_rpm = speed_counts_to_rpm(s_mc.speed_fb_ma600);
    out->ma600_speed_raw = s_mc.ma600_speed_raw;
    out->speed_fb_source = CTRL_SPD_FB_SOURCE;
    out->speed_err_rpm = s_mc.speed_err_rpm;
    out->speed_iq_cmd = s_mc.speed_iq_ref;
    out->speed_pi_integral = s_mc.speed_pi.integral;
    out->id = s_mc.current_dq.d;
    out->iq = s_mc.current_dq.q;
    out->vd = s_mc.voltage_dq.d;
    out->vq = s_mc.voltage_dq.q;
    out->voltage_theta = s_mc.voltage_theta;
    out->v_limited = s_mc.voltage_limited;
    out->duty_u = (uint16_t)duty_u;
    out->duty_v = (uint16_t)duty_v;
    out->duty_w = (uint16_t)duty_w;
    out->pwm_safe = pwm_is_safe();
    out->pwm_running = (uint8_t)((pwm_out != 0U) && (pwm_is_running() != 0U));
    out->check = s_mc.check;
    out->command_apply_count = s_mc.command_apply_count;
    out->command_enable = s_mc.command.enable;
    out->command_control_mode = s_mc.command.control_mode;
    out->command_vf_voltage = s_mc.command.vf_voltage;
    out->command_open_loop_speed_ref = s_mc.command.open_loop_speed_ref;
    out->command_speed_ref_rpm = s_mc.command.speed_ref_rpm;
    out->command_iq_limit = s_mc.command.iq_limit;
    out->command_current_v_limit = s_mc.command.current_v_limit;
    out->command_voltage_theta_offset = s_mc.command.voltage_theta_offset;
    out->command_speed_kp = s_mc.command.speed_kp;
    out->command_speed_ki = s_mc.command.speed_ki;
}

static void copy_watch_to_volatile(volatile MotorControlWatch_t* dst,
                                   const MotorControlWatch_t* src)
{
    dst->state = src->state;
    dst->control_mode = src->control_mode;
    dst->fault_reason = src->fault_reason;
    dst->enable = src->enable;
    dst->slow_loop_count = src->slow_loop_count;
    dst->fast_loop_count = src->fast_loop_count;
    dst->adc_sample_count = src->adc_sample_count;
    dst->encoder_raw = src->encoder_raw;
    dst->encoder_elec = src->encoder_elec;
    dst->encoder_delta = src->encoder_delta;
    dst->encoder_pos = src->encoder_pos;
    dst->encoder_age = src->encoder_age;
    dst->encoder_ok = src->encoder_ok;
    dst->align_done = src->align_done;
    dst->align_ticks = src->align_ticks;
    dst->align_theta = src->align_theta;
    dst->align_raw = src->align_raw;
    dst->align_zero_trim = src->align_zero_trim;
    dst->align_encoder_elec = src->align_encoder_elec;
    dst->align_stage = src->align_stage;
    dst->align_pull_delta = src->align_pull_delta;
    dst->align_sample_count = src->align_sample_count;
    dst->align_delta_sum = src->align_delta_sum;
    dst->iu_cnt = src->iu_cnt;
    dst->iv_cnt = src->iv_cnt;
    dst->iw_cnt = src->iw_cnt;
    dst->i_sum = src->i_sum;
    dst->id_ref = src->id_ref;
    dst->iq_ref = src->iq_ref;
    dst->speed_ref = src->speed_ref;
    dst->speed_ref_rpm = src->speed_ref_rpm;
    dst->speed_fb = src->speed_fb;
    dst->speed_fb_rpm = src->speed_fb_rpm;
    dst->speed_fb_diff = src->speed_fb_diff;
    dst->speed_fb_diff_rpm = src->speed_fb_diff_rpm;
    dst->speed_fb_ma600 = src->speed_fb_ma600;
    dst->speed_fb_ma600_rpm = src->speed_fb_ma600_rpm;
    dst->ma600_speed_raw = src->ma600_speed_raw;
    dst->speed_fb_source = src->speed_fb_source;
    dst->speed_err_rpm = src->speed_err_rpm;
    dst->speed_iq_cmd = src->speed_iq_cmd;
    dst->speed_pi_integral = src->speed_pi_integral;
    dst->id = src->id;
    dst->iq = src->iq;
    dst->vd = src->vd;
    dst->vq = src->vq;
    dst->voltage_theta = src->voltage_theta;
    dst->v_limited = src->v_limited;
    dst->duty_u = src->duty_u;
    dst->duty_v = src->duty_v;
    dst->duty_w = src->duty_w;
    dst->pwm_safe = src->pwm_safe;
    dst->pwm_running = src->pwm_running;
    dst->check.ma600_ok = src->check.ma600_ok;
    dst->check.current_ok = src->check.current_ok;
    dst->check.pwm_off_safe = src->check.pwm_off_safe;
    dst->check.ready_closed_loop = src->check.ready_closed_loop;
    dst->command_apply_count = src->command_apply_count;
    dst->command_enable = src->command_enable;
    dst->command_control_mode = src->command_control_mode;
    dst->command_vf_voltage = src->command_vf_voltage;
    dst->command_open_loop_speed_ref = src->command_open_loop_speed_ref;
    dst->command_speed_ref_rpm = src->command_speed_ref_rpm;
    dst->command_iq_limit = src->command_iq_limit;
    dst->command_current_v_limit = src->command_current_v_limit;
    dst->command_voltage_theta_offset = src->command_voltage_theta_offset;
    dst->command_speed_kp = src->command_speed_kp;
    dst->command_speed_ki = src->command_speed_ki;
}

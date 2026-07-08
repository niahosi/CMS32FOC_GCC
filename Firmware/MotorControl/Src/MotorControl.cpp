#include "MotorControl.h"

#include "Axis.hpp"

extern "C" {
#include "foc_bsp.h"
}

namespace {

cms32::control::Axis g_axis;

static void copy_watch_to_volatile(volatile MotorControlWatch_t* dst,
                                   const MotorControlWatch_t& src)
{
    dst->state = src.state;
    dst->control_mode = src.control_mode;
    dst->fault_reason = src.fault_reason;
    dst->enable = src.enable;
    dst->slow_loop_count = src.slow_loop_count;
    dst->fast_loop_count = src.fast_loop_count;
    dst->adc_sample_count = src.adc_sample_count;
    dst->encoder_raw = src.encoder_raw;
    dst->encoder_elec = src.encoder_elec;
    dst->encoder_delta = src.encoder_delta;
    dst->encoder_pos = src.encoder_pos;
    dst->encoder_age = src.encoder_age;
    dst->encoder_ok = src.encoder_ok;
    dst->iu_cnt = src.iu_cnt;
    dst->iv_cnt = src.iv_cnt;
    dst->iw_cnt = src.iw_cnt;
    dst->i_sum = src.i_sum;
    dst->id_ref = src.id_ref;
    dst->iq_ref = src.iq_ref;
    dst->speed_ref = src.speed_ref;
    dst->speed_fb = src.speed_fb;
    dst->id = src.id;
    dst->iq = src.iq;
    dst->vd = src.vd;
    dst->vq = src.vq;
    dst->v_limited = src.v_limited;
    dst->duty_u = src.duty_u;
    dst->duty_v = src.duty_v;
    dst->duty_w = src.duty_w;
    dst->pwm_safe = src.pwm_safe;
    dst->pwm_running = src.pwm_running;
    dst->check.ma600_ok = src.check.ma600_ok;
    dst->check.current_ok = src.check.current_ok;
    dst->check.pwm_off_safe = src.check.pwm_off_safe;
    dst->check.ready_closed_loop = src.check.ready_closed_loop;
    dst->command_apply_count = src.command_apply_count;
    dst->command_enable = src.command_enable;
    dst->command_control_mode = src.command_control_mode;
    dst->command_vf_voltage = src.command_vf_voltage;
    dst->command_open_loop_speed_ref = src.command_open_loop_speed_ref;
}

} // namespace

extern "C" void MotorControl_Init(void)
{
    g_axis.init();
}

extern "C" void MotorControl_ApplyCommand(const volatile MotorControlCommand_t* command)
{
    if (command == nullptr)
    {
        return;
    }
    g_axis.applyCommand(*command);
}

extern "C" void MotorControl_RunSlowLoop(void)
{
    g_axis.runSlowLoop();
}

extern "C" uint8_t MotorControl_FastLoopFromAdcIrq(void)
{
    const uint8_t sample_ready = bsp_adc_irq();
    if ((sample_ready != 0U) && g_axis.controller().enabled())
    {
        g_axis.updateEncoderFast();
        g_axis.runFastLoop();
    }
    return sample_ready;
}

extern "C" void MotorControl_GetWatch(MotorControlWatch_t* out)
{
    if (out == nullptr)
    {
        return;
    }

    g_axis.fillWatch(*out);
}

extern "C" void MotorControl_UpdateWatch(volatile MotorControlWatch_t* out)
{
    MotorControlWatch_t snapshot;

    if (out == nullptr)
    {
        return;
    }

    g_axis.fillWatch(snapshot);
    copy_watch_to_volatile(out, snapshot);
}

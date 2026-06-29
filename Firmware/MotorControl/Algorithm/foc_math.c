#include "foc_math.h"

#define FOC_PI_INTEGRAL_LIMIT 32767L

static const int16_t s_sin_quarter[257] = {
    0,     201,   402,   603,   804,   1005,  1206,  1407,  1608,  1809,  2009,  2210,  2410,
    2611,  2811,  3012,  3212,  3412,  3612,  3811,  4011,  4210,  4410,  4609,  4808,  5007,
    5205,  5404,  5602,  5800,  5998,  6195,  6393,  6590,  6786,  6983,  7179,  7375,  7571,
    7767,  7962,  8157,  8351,  8545,  8739,  8933,  9126,  9319,  9512,  9704,  9896,  10087,
    10278, 10469, 10659, 10849, 11039, 11228, 11417, 11605, 11793, 11980, 12167, 12353, 12539,
    12725, 12910, 13094, 13279, 13462, 13645, 13828, 14010, 14191, 14372, 14553, 14732, 14912,
    15090, 15269, 15446, 15623, 15800, 15976, 16151, 16325, 16499, 16673, 16846, 17018, 17189,
    17360, 17530, 17700, 17869, 18037, 18204, 18371, 18537, 18703, 18868, 19032, 19195, 19357,
    19519, 19680, 19841, 20000, 20159, 20317, 20475, 20631, 20787, 20942, 21096, 21250, 21403,
    21554, 21705, 21856, 22005, 22154, 22301, 22448, 22594, 22739, 22884, 23027, 23170, 23311,
    23452, 23592, 23731, 23870, 24007, 24143, 24279, 24413, 24547, 24680, 24811, 24942, 25072,
    25201, 25329, 25456, 25582, 25708, 25832, 25955, 26077, 26198, 26319, 26438, 26556, 26674,
    26790, 26905, 27019, 27133, 27245, 27356, 27466, 27575, 27683, 27790, 27896, 28001, 28105,
    28208, 28310, 28411, 28510, 28609, 28706, 28803, 28898, 28992, 29085, 29177, 29268, 29358,
    29447, 29534, 29621, 29706, 29791, 29874, 29956, 30037, 30117, 30195, 30273, 30349, 30424,
    30498, 30571, 30643, 30714, 30783, 30852, 30919, 30985, 31050, 31113, 31176, 31237, 31297,
    31356, 31414, 31470, 31526, 31580, 31633, 31685, 31736, 31785, 31833, 31880, 31926, 31971,
    32014, 32057, 32098, 32137, 32176, 32213, 32250, 32285, 32318, 32351, 32382, 32412, 32441,
    32469, 32495, 32521, 32545, 32567, 32589, 32609, 32628, 32646, 32663, 32678, 32692, 32705,
    32717, 32728, 32737, 32745, 32752, 32757, 32761, 32765, 32766, 32767};

static int32_t scale_down_s32(int32_t value, uint8_t shift)
{
    if (shift == 0U)
    {
        return value;
    }
    if (shift >= 31U)
    {
        return 0;
    }
    if (value >= 0)
    {
        return value >> shift;
    }
    return -((-value) >> shift);
}

static int32_t abs_s16_to_s32(int16_t value)
{
    return (value < 0) ? -(int32_t)value : (int32_t)value;
}

int16_t foc_sin_q15(uint16_t angle)
{
    uint16_t phase = (uint16_t)(angle & 0x3FFFU);
    uint16_t index = (uint16_t)(phase >> 6);
    uint16_t frac = (uint16_t)(phase & 0x3FU);
    uint8_t quadrant = (uint8_t)(angle >> 14);
    int32_t y0;
    int32_t y1;
    int32_t value;

    if ((quadrant == 0U) || (quadrant == 2U))
    {
        y0 = s_sin_quarter[index];
        y1 = s_sin_quarter[index + 1U];
    }
    else
    {
        y0 = s_sin_quarter[256U - index];
        y1 = s_sin_quarter[255U - index];
    }

    value = y0 + (((y1 - y0) * (int32_t)frac) >> 6);
    if (quadrant >= 2U)
    {
        value = -value;
    }
    return (int16_t)value;
}

int16_t foc_cos_q15(uint16_t angle)
{
    return foc_sin_q15((uint16_t)(angle + 16384U));
}

int16_t foc_clamp_s16(int16_t value, int16_t min, int16_t max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

int32_t foc_clamp_s32(int32_t value, int32_t min, int32_t max)
{
    if (value < min)
    {
        return min;
    }
    if (value > max)
    {
        return max;
    }
    return value;
}

FocPhaseCurrent_t foc_phase_sum_correct(FocPhaseCurrent_t current)
{
    int16_t mid = (int16_t)(((int32_t)current.u + (int32_t)current.v + (int32_t)current.w) / 3);
    FocPhaseCurrent_t corrected = {
        .u = (int16_t)(current.u - mid),
        .v = (int16_t)(current.v - mid),
        .w = (int16_t)(current.w - mid),
    };
    return corrected;
}

FocAlphaBeta_t foc_clarke_2phase(int16_t iu, int16_t iv)
{
    FocAlphaBeta_t out = {
        .alpha = iu,
        .beta = (int16_t)(((int32_t)iu * 9459 + (int32_t)iv * 18919) >> 14),
    };
    return out;
}

FocAlphaBeta_t foc_clarke_3phase(FocPhaseCurrent_t current)
{
    FocPhaseCurrent_t corrected = foc_phase_sum_correct(current);
    return foc_clarke_2phase(corrected.u, corrected.v);
}

FocDq_t foc_park(FocAlphaBeta_t input, uint16_t theta)
{
    int16_t sin_theta = foc_sin_q15(theta);
    int16_t cos_theta = foc_cos_q15(theta);
    FocDq_t out = {
        .d = (int16_t)(((int32_t)input.alpha * cos_theta + (int32_t)input.beta * sin_theta) >> 15),
        .q = (int16_t)((-(int32_t)input.alpha * sin_theta + (int32_t)input.beta * cos_theta) >> 15),
    };
    return out;
}

FocAlphaBeta_t foc_inv_park(FocDq_t input, uint16_t theta)
{
    int16_t sin_theta = foc_sin_q15(theta);
    int16_t cos_theta = foc_cos_q15(theta);
    FocAlphaBeta_t out = {
        .alpha = (int16_t)(((int32_t)input.d * cos_theta - (int32_t)input.q * sin_theta) >> 15),
        .beta = (int16_t)(((int32_t)input.d * sin_theta + (int32_t)input.q * cos_theta) >> 15),
    };
    return out;
}

void foc_pi_init(FocPi_t* pi, int16_t kp, int16_t ki, int16_t output_min, int16_t output_max,
                 uint8_t shift)
{
    if (pi == 0)
    {
        return;
    }
    pi->kp = kp;
    pi->ki = ki;
    pi->error = 0;
    pi->error_prev = 0;
    pi->integral = 0;
    pi->output = 0;
    pi->output_min = output_min;
    pi->output_max = output_max;
    pi->shift = shift;
}

void foc_pi_reset(FocPi_t* pi)
{
    if (pi == 0)
    {
        return;
    }
    pi->error = 0;
    pi->error_prev = 0;
    pi->integral = 0;
    pi->output = 0;
}

void foc_pi_set_gains(FocPi_t* pi, int16_t kp, int16_t ki, int16_t output_min,
                      int16_t output_max, uint8_t shift)
{
    if (pi == 0)
    {
        return;
    }
    pi->kp = kp;
    pi->ki = ki;
    pi->output_min = output_min;
    pi->output_max = output_max;
    pi->shift = shift;
}

int16_t foc_pi_update(FocPi_t* pi, int16_t ref, int16_t feedback)
{
    int32_t integral_new;
    int32_t output_raw;
    int32_t output_unclamped;
    int32_t output;

    if (pi == 0)
    {
        return 0;
    }

    pi->error = (int16_t)foc_clamp_s32((int32_t)ref - (int32_t)feedback, -32768, 32767);
    integral_new = foc_clamp_s32(pi->integral + pi->error, -FOC_PI_INTEGRAL_LIMIT,
                                 FOC_PI_INTEGRAL_LIMIT);

    output_raw = (int32_t)pi->kp * pi->error + (int32_t)pi->ki * integral_new;
    output_unclamped = scale_down_s32(output_raw, pi->shift);
    output = foc_clamp_s32(output_unclamped, pi->output_min, pi->output_max);

    if ((output == output_unclamped) || (pi->ki == 0) ||
        ((output_unclamped > pi->output_max) && (pi->error < 0)) ||
        ((output_unclamped < pi->output_min) && (pi->error > 0)))
    {
        pi->integral = integral_new;
    }

    output_raw = (int32_t)pi->kp * pi->error + (int32_t)pi->ki * pi->integral;
    output_unclamped = scale_down_s32(output_raw, pi->shift);
    output = foc_clamp_s32(output_unclamped, pi->output_min, pi->output_max);

    pi->output = (int16_t)output;
    pi->error_prev = pi->error;
    return pi->output;
}

uint8_t foc_limit_dq(FocDq_t* voltage, int16_t limit)
{
    int32_t abs_d;
    int32_t abs_q;
    int32_t max_abs;
    int32_t min_abs;
    int32_t mag;
    int32_t limit_abs;

    if (voltage == 0)
    {
        return 0U;
    }

    limit_abs = abs_s16_to_s32(limit);
    if (limit_abs <= 0)
    {
        voltage->d = 0;
        voltage->q = 0;
        return 1U;
    }

    abs_d = abs_s16_to_s32(voltage->d);
    abs_q = abs_s16_to_s32(voltage->q);
    max_abs = (abs_d >= abs_q) ? abs_d : abs_q;
    min_abs = (abs_d >= abs_q) ? abs_q : abs_d;
    mag = max_abs + (min_abs >> 1);
    if (mag <= limit_abs)
    {
        return 0U;
    }

    voltage->d = (int16_t)(((int32_t)voltage->d * limit_abs) / mag);
    voltage->q = (int16_t)(((int32_t)voltage->q * limit_abs) / mag);
    return 1U;
}

FocDuty_t foc_svpwm(FocAlphaBeta_t voltage, uint16_t vdc, uint16_t duty_min,
                    uint16_t duty_max)
{
    int32_t vu;
    int32_t vv;
    int32_t vw;
    int32_t vmax;
    int32_t vmin;
    int32_t vzero;
    int32_t center;
    FocDuty_t out;

    center = (vdc == 0U) ? 0 : ((int32_t)vdc / 2);
    vu = voltage.alpha;
    vv = (-(int32_t)voltage.alpha / 2) + ((int32_t)voltage.beta * 866 / 1000);
    vw = (-(int32_t)voltage.alpha / 2) - ((int32_t)voltage.beta * 866 / 1000);

    vmax = vu;
    if (vv > vmax)
    {
        vmax = vv;
    }
    if (vw > vmax)
    {
        vmax = vw;
    }

    vmin = vu;
    if (vv < vmin)
    {
        vmin = vv;
    }
    if (vw < vmin)
    {
        vmin = vw;
    }

    vzero = -((vmax + vmin) / 2);
    out.u = (uint16_t)foc_clamp_s32(center + vu + vzero, duty_min, duty_max);
    out.v = (uint16_t)foc_clamp_s32(center + vv + vzero, duty_min, duty_max);
    out.w = (uint16_t)foc_clamp_s32(center + vw + vzero, duty_min, duty_max);
    return out;
}

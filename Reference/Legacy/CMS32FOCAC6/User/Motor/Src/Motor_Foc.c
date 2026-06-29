/**
 * @file Motor_Foc.c
 * @brief FOC 算法实现
 * @details 实现磁场定向控制（FOC）所需的算法函数
 */

#include "Motor_Foc.h"

#define FOC_PI_INTEGRAL_LIMIT 32767l

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

static int32_t ScaleDownS32(int32_t value, uint8_t shift)
{
    if (shift == 0u)
    {
        return value;
    }
    if (shift >= 31u)
    {
        return 0;
    }
    if (value >= 0)
    {
        return value >> shift;
    }

    return -((-value) >> shift);
}

static int32_t AbsS16ToS32(int16_t value)
{
    if (value < 0)
    {
        return -(int32_t)value;
    }

    return value;
}

int16_t Foc_Sin(uint16_t angle)
{
    uint16_t phase = (uint16_t)(angle & 0x3FFFu);
    uint16_t index = (uint16_t)(phase >> 6);
    uint16_t frac = (uint16_t)(phase & 0x3Fu);
    uint8_t quadrant = (uint8_t)(angle >> 14);
    int32_t y0;
    int32_t y1;
    int32_t value;

    if ((quadrant == 0u) || (quadrant == 2u))
    {
        y0 = s_sin_quarter[index];
        y1 = s_sin_quarter[index + 1u];
    }
    else
    {
        y0 = s_sin_quarter[256u - index];
        y1 = s_sin_quarter[255u - index];
    }

    value = y0 + (((y1 - y0) * (int32_t)frac) >> 6);

    if (quadrant >= 2u)
    {
        value = -value;
    }

    return (int16_t)value;
}

int16_t Foc_Cos(uint16_t angle)
{
    // cos(θ) = sin(θ + 90°)
    // 90° 对应 16384 (65536/4)
    return Foc_Sin(angle + 16384);
}

void Foc_ClarkeTransform(int16_t iu, int16_t iv, int16_t iw, int16_t* i_alpha, int16_t* i_beta)
{
    (void)iw;

    // Clarke 变换公式：
    // i_alpha = iu
    // i_beta = (iu + 2 * iv) / sqrt(3)
    // 使用定点运算：5773/10000 ≈ 1/sqrt(3)
    *i_alpha = iu;
    *i_beta = (int16_t)((iu + 2 * iv) * 5773 / 10000);
}

void Foc_ParkTransform(int16_t i_alpha, int16_t i_beta, uint16_t theta, int16_t* id, int16_t* iq)
{
    int16_t sin_theta = Foc_Sin(theta);
    int16_t cos_theta = Foc_Cos(theta);

    // Park 变换公式：
    // id = i_alpha * cos(θ) + i_beta * sin(θ)
    // iq = -i_alpha * sin(θ) + i_beta * cos(θ)
    *id = (int16_t)((i_alpha * cos_theta + i_beta * sin_theta) >> 15);
    *iq = (int16_t)((-i_alpha * sin_theta + i_beta * cos_theta) >> 15);
}

void Foc_InvParkTransform(int16_t vd, int16_t vq, uint16_t theta, int16_t* v_alpha, int16_t* v_beta)
{
    int16_t sin_theta = Foc_Sin(theta);
    int16_t cos_theta = Foc_Cos(theta);

    // 反 Park 变换公式：
    // v_alpha = vd * cos(θ) - vq * sin(θ)
    // v_beta = vd * sin(θ) + vq * cos(θ)
    *v_alpha = (int16_t)((vd * cos_theta - vq * sin_theta) >> 15);
    *v_beta = (int16_t)((vd * sin_theta + vq * cos_theta) >> 15);
}

int16_t Foc_ClampS16(int16_t value, int16_t min, int16_t max)
{
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }
    else
    {
        return value;
    }
}

int32_t Foc_ClampS32(int32_t value, int32_t min, int32_t max)
{
    if (value < min)
    {
        return min;
    }
    else if (value > max)
    {
        return max;
    }
    else
    {
        return value;
    }
}

void Foc_PiInit(Foc_PiController_t* pi, int16_t kp, int16_t ki, int16_t output_min,
                int16_t output_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->error = 0;
    pi->error_prev = 0;
    pi->integral = 0;
    pi->output = 0;
    pi->output_min = output_min;
    pi->output_max = output_max;
    pi->shift = 0u;
}

void Foc_PiSetShift(Foc_PiController_t* pi, uint8_t shift)
{
    if (pi == 0)
    {
        return;
    }

    pi->shift = shift;
}

int16_t Foc_PiUpdate(Foc_PiController_t* pi, int16_t ref, int16_t fb)
{
    int32_t integral_new;
    int32_t output_raw;
    int32_t output_unclamped;
    int32_t output;

    if (pi == 0)
    {
        return 0;
    }

    pi->error = (int16_t)Foc_ClampS32((int32_t)ref - (int32_t)fb, -32768, 32767);

    integral_new = pi->integral + pi->error;
    integral_new = Foc_ClampS32(integral_new, -FOC_PI_INTEGRAL_LIMIT, FOC_PI_INTEGRAL_LIMIT);

    output_raw = (int32_t)pi->kp * pi->error + (int32_t)pi->ki * integral_new;
    output_unclamped = ScaleDownS32(output_raw, pi->shift);
    output = Foc_ClampS32(output_unclamped, pi->output_min, pi->output_max);

    /*
     * Anti-windup: when the output is saturated, only accept integration that
     * helps pull the output back from the clamp.
     */
    if ((output == output_unclamped) || (pi->ki == 0) ||
        ((output_unclamped > pi->output_max) && (pi->error < 0)) ||
        ((output_unclamped < pi->output_min) && (pi->error > 0)))
    {
        pi->integral = integral_new;
    }

    output_raw = (int32_t)pi->kp * pi->error + (int32_t)pi->ki * pi->integral;
    output_unclamped = ScaleDownS32(output_raw, pi->shift);
    output = Foc_ClampS32(output_unclamped, pi->output_min, pi->output_max);

    pi->output = (int16_t)output;
    pi->error_prev = pi->error;

    return pi->output;
}

uint8_t Foc_LimitDq(int16_t* vd, int16_t* vq, int16_t limit)
{
    int32_t abs_d;
    int32_t abs_q;
    int32_t max_abs;
    int32_t min_abs;
    int32_t mag;
    int32_t limit_abs;

    if ((vd == 0) || (vq == 0))
    {
        return 0u;
    }

    limit_abs = AbsS16ToS32(limit);
    if (limit_abs <= 0)
    {
        *vd = 0;
        *vq = 0;
        return 1u;
    }

    abs_d = AbsS16ToS32(*vd);
    abs_q = AbsS16ToS32(*vq);
    if (abs_d >= abs_q)
    {
        max_abs = abs_d;
        min_abs = abs_q;
    }
    else
    {
        max_abs = abs_q;
        min_abs = abs_d;
    }

    /*
     * Conservative vector magnitude approximation:
     * max(abs(d), abs(q)) + min(abs(d), abs(q)) / 2.
     */
    mag = max_abs + (min_abs >> 1);
    if (mag <= limit_abs)
    {
        return 0u;
    }

    *vd = (int16_t)(((int32_t)(*vd) * limit_abs) / mag);
    *vq = (int16_t)(((int32_t)(*vq) * limit_abs) / mag);
    return 1u;
}

void Foc_Svpwm(int16_t v_alpha, int16_t v_beta, uint16_t vdc, uint16_t* duty_u, uint16_t* duty_v,
               uint16_t* duty_w)
{
    int32_t vu;
    int32_t vv;
    int32_t vw;
    int32_t vmax;
    int32_t vmin;
    int32_t vzero;
    int32_t center;

    if ((duty_u == 0) || (duty_v == 0) || (duty_w == 0))
    {
        return;
    }

    if (vdc == 0u)
    {
        center = PWM_PERIOD / 2;
    }
    else
    {
        center = (int32_t)vdc / 2;
    }

    vu = v_alpha;
    vv = (-(int32_t)v_alpha / 2) + ((int32_t)v_beta * 866 / 1000);
    vw = (-(int32_t)v_alpha / 2) - ((int32_t)v_beta * 866 / 1000);

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

    *duty_u = (uint16_t)Foc_ClampS32(center + vu + vzero, PWM_DUTY_MIN, PWM_DUTY_MAX);
    *duty_v = (uint16_t)Foc_ClampS32(center + vv + vzero, PWM_DUTY_MIN, PWM_DUTY_MAX);
    *duty_w = (uint16_t)Foc_ClampS32(center + vw + vzero, PWM_DUTY_MIN, PWM_DUTY_MAX);
}

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int16_t u;
    int16_t v;
    int16_t w;
} FocPhaseCurrent_t;

typedef struct
{
    int16_t alpha;
    int16_t beta;
} FocAlphaBeta_t;

typedef struct
{
    int16_t d;
    int16_t q;
} FocDq_t;

typedef struct
{
    uint16_t u;
    uint16_t v;
    uint16_t w;
} FocDuty_t;

typedef struct
{
    int16_t kp;
    int16_t ki;
    int16_t error;
    int16_t error_prev;
    int32_t integral;
    int16_t output;
    int16_t output_min;
    int16_t output_max;
    uint8_t shift;
} FocPi_t;

int16_t foc_sin_q15(uint16_t angle);
int16_t foc_cos_q15(uint16_t angle);

int16_t foc_clamp_s16(int16_t value, int16_t min, int16_t max);
int32_t foc_clamp_s32(int32_t value, int32_t min, int32_t max);

FocPhaseCurrent_t foc_phase_sum_correct(FocPhaseCurrent_t current);
FocAlphaBeta_t foc_clarke_2phase(int16_t iu, int16_t iv);
FocAlphaBeta_t foc_clarke_3phase(FocPhaseCurrent_t current);
FocDq_t foc_park(FocAlphaBeta_t input, uint16_t theta);
FocAlphaBeta_t foc_inv_park(FocDq_t input, uint16_t theta);

void foc_pi_init(FocPi_t* pi, int16_t kp, int16_t ki, int16_t output_min, int16_t output_max,
                 uint8_t shift);
void foc_pi_reset(FocPi_t* pi);
void foc_pi_set_gains(FocPi_t* pi, int16_t kp, int16_t ki, int16_t output_min,
                      int16_t output_max, uint8_t shift);
int16_t foc_pi_update(FocPi_t* pi, int16_t ref, int16_t feedback);

uint8_t foc_limit_dq(FocDq_t* voltage, int16_t limit);
FocDuty_t foc_svpwm(FocAlphaBeta_t voltage, uint16_t vdc, uint16_t duty_min,
                    uint16_t duty_max);

#ifdef __cplusplus
}
#endif

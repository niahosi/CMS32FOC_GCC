#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 三相电流，内部 ADC count 缩放单位。 */
typedef struct
{
    int16_t u;
    int16_t v;
    int16_t w;
} FocPhaseCurrent_t;

/** @brief alpha/beta 静止坐标系量。 */
typedef struct
{
    int16_t alpha;
    int16_t beta;
} FocAlphaBeta_t;

/** @brief d/q 旋转坐标系量。 */
typedef struct
{
    int16_t d;
    int16_t q;
} FocDq_t;

/** @brief 三相 SVPWM duty count。 */
typedef struct
{
    uint16_t u;
    uint16_t v;
    uint16_t w;
} FocDuty_t;

/** @brief 定点 PI 控制器状态。 */
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

/** @brief 16-bit 周期角的 Q15 sin。 */
int16_t foc_sin_q15(uint16_t angle);
/** @brief 16-bit 周期角的 Q15 cos。 */
int16_t foc_cos_q15(uint16_t angle);

/** @brief int16 限幅。 */
int16_t foc_clamp_s16(int16_t value, int16_t min, int16_t max);
/** @brief int32 限幅。 */
int32_t foc_clamp_s32(int32_t value, int32_t min, int32_t max);

/** @brief 去除三相公共和误差，用于采样诊断或补偿。 */
FocPhaseCurrent_t foc_phase_sum_correct(FocPhaseCurrent_t current);
/** @brief 两相电流 Clarke 变换。 */
FocAlphaBeta_t foc_clarke_2phase(int16_t iu, int16_t iv);
/** @brief 三相电流 Clarke 变换。 */
FocAlphaBeta_t foc_clarke_3phase(FocPhaseCurrent_t current);
/** @brief Park 变换，theta 为 16-bit 周期电角度。 */
FocDq_t foc_park(FocAlphaBeta_t input, uint16_t theta);
/** @brief 反 Park 变换，theta 为 16-bit 周期电角度。 */
FocAlphaBeta_t foc_inv_park(FocDq_t input, uint16_t theta);

/** @brief 初始化定点 PI 控制器。 */
void foc_pi_init(FocPi_t* pi, int16_t kp, int16_t ki, int16_t output_min, int16_t output_max,
                 uint8_t shift);
/** @brief 清零 PI 内部误差、积分和输出。 */
void foc_pi_reset(FocPi_t* pi);
/** @brief 更新 PI 增益、输出限幅和定点 shift。 */
void foc_pi_set_gains(FocPi_t* pi, int16_t kp, int16_t ki, int16_t output_min,
                      int16_t output_max, uint8_t shift);
/** @brief 执行一次 PI 更新，返回限幅后的输出。 */
int16_t foc_pi_update(FocPi_t* pi, int16_t ref, int16_t feedback);

/** @brief 限制 dq 电压矢量幅值，返回 1 表示发生限幅。 */
uint8_t foc_limit_dq(FocDq_t* voltage, int16_t limit);
/** @brief 将 alpha/beta 电压转换为三相 SVPWM duty count。 */
FocDuty_t foc_svpwm(FocAlphaBeta_t voltage, uint16_t vdc, uint16_t duty_min,
                    uint16_t duty_max);

#ifdef __cplusplus
}
#endif

/**
 * @file Motor_Foc.h
 * @brief FOC 算法接口
 * @details 提供磁场定向控制（FOC）所需的算法函数，包括：
 *          - sin/cos 查找表
 *          - Clarke 变换
 *          - Park 变换
 *          - 反 Park 变换
 *          - 电流环 PI
 *          - SVPWM
 */

#pragma once
#include <stdint.h>
#include "Board_PWM.h"

/**
 * @brief PI 控制器状态结构体。
 * @details 使用定点运算，output = (kp * error + integral) >> shift。
 */
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
} Foc_PiController_t;

/**
 * @brief 计算正弦值。
 * @param[in] angle 电角度，0~65535 对应一电周期。
 * @return Q15 正弦值，范围约为 -32767~32767。
 */
int16_t Foc_Sin(uint16_t angle);

/**
 * @brief 计算余弦值。
 * @param[in] angle 电角度，0~65535 对应一电周期。
 * @return Q15 余弦值，范围约为 -32767~32767。
 */
int16_t Foc_Cos(uint16_t angle);

/**
 * @brief 执行 Clarke 变换。
 * @param[in] iu U 相电流，单位 ADC count。
 * @param[in] iv V 相电流，单位 ADC count。
 * @param[in] iw W 相电流，单位 ADC count。
 * @param[out] i_alpha alpha 轴电流输出。
 * @param[out] i_beta beta 轴电流输出。
 */
void Foc_ClarkeTransform(int16_t iu, int16_t iv, int16_t iw, int16_t* i_alpha, int16_t* i_beta);

/**
 * @brief 执行 Park 变换。
 * @param[in] i_alpha alpha 轴电流。
 * @param[in] i_beta beta 轴电流。
 * @param[in] theta 电角度，0~65535 对应一电周期。
 * @param[out] id d 轴电流输出。
 * @param[out] iq q 轴电流输出。
 */
void Foc_ParkTransform(int16_t i_alpha, int16_t i_beta, uint16_t theta, int16_t* id, int16_t* iq);

/**
 * @brief 执行反 Park 变换。
 * @param[in] vd d 轴电压给定。
 * @param[in] vq q 轴电压给定。
 * @param[in] theta 电角度，0~65535 对应一电周期。
 * @param[out] v_alpha alpha 轴电压输出。
 * @param[out] v_beta beta 轴电压输出。
 */
void Foc_InvParkTransform(int16_t vd, int16_t vq, uint16_t theta, int16_t* v_alpha,
                          int16_t* v_beta);

/**
 * @brief 初始化 PI 控制器。
 * @param[out] pi PI 控制器对象。
 * @param[in] kp 比例系数。
 * @param[in] ki 积分系数。
 * @param[in] output_min 输出下限。
 * @param[in] output_max 输出上限。
 */
void Foc_PiInit(Foc_PiController_t* pi, int16_t kp, int16_t ki, int16_t output_min,
                int16_t output_max);

/**
 * @brief Setter: 设置 PI 输出定点右移位数。
 * @param[in,out] pi PI 控制器对象。
 * @param[in] shift 输出右移位数。
 */
void Foc_PiSetShift(Foc_PiController_t* pi, uint8_t shift);

/**
 * @brief 更新 PI 控制器。
 * @param[in,out] pi PI 控制器对象。
 * @param[in] ref 给定值。
 * @param[in] fb 反馈值。
 * @return 限幅后的 PI 输出。
 */
int16_t Foc_PiUpdate(Foc_PiController_t* pi, int16_t ref, int16_t fb);

/**
 * @brief 对 int16_t 数值限幅。
 * @param[in] value 输入值。
 * @param[in] min 最小值。
 * @param[in] max 最大值。
 * @return 限幅后的值。
 */
int16_t Foc_ClampS16(int16_t value, int16_t min, int16_t max);

/**
 * @brief 对 int32_t 数值限幅。
 * @param[in] value 输入值。
 * @param[in] min 最小值。
 * @param[in] max 最大值。
 * @return 限幅后的值。
 */
int32_t Foc_ClampS32(int32_t value, int32_t min, int32_t max);

/**
 * @brief 对 d/q 轴电压矢量做限幅。
 * @param[in,out] vd d 轴电压指针。
 * @param[in,out] vq q 轴电压指针。
 * @param[in] limit 电压幅值近似限幅。
 * @return 1 表示发生限幅，0 表示未限幅。
 */
uint8_t Foc_LimitDq(int16_t* vd, int16_t* vq, int16_t limit);

/**
 * @brief 执行 SVPWM 计算。
 * @param[in] v_alpha alpha 轴电压。
 * @param[in] v_beta beta 轴电压。
 * @param[in] vdc PWM 可用电压/计数标尺。
 * @param[out] duty_u U 相占空比计数。
 * @param[out] duty_v V 相占空比计数。
 * @param[out] duty_w W 相占空比计数。
 */
void Foc_Svpwm(int16_t v_alpha, int16_t v_beta, uint16_t vdc, uint16_t* duty_u, uint16_t* duty_v,
               uint16_t* duty_w);

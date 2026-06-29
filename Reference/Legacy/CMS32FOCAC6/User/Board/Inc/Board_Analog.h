/**
 * @file Board_Analog.h
 * @brief 板级模拟采样接口。
 * @details 提供 ADC/PGA 初始化、电流零漂校准、PWM 触发采样和电流缓存读取。
 */

#pragma once

#include <stdint.h>
#include "Config.h"

/**
 * @brief 初始化 ADC、ADCLDO 和三相电流 PGA。
 */
void Board_InitAdc(void);

/**
 * @brief 执行一次软件触发三相电流采样。
 * @details 主要用于 PWM 关闭时的零漂校准。
 */
void Board_SampleAdc(void);

/**
 * @brief 采集静态电流零漂。
 * @param[in] samples 平均采样次数。
 */
void Board_CalibCurrentOffset(uint16_t samples);

/**
 * @brief 在 PWM 运行配置下重新采集电流零漂。
 * @param[in] samples 平均采样次数。
 */
void Board_CalibCurrentOffsetPwm(uint16_t samples);

/**
 * @brief 根据最新 ADC 原始值更新三相逻辑电流。
 */
void Board_UpdateCurrent(void);

/**
 * @brief 初始化 PWM 触发 ADC 同步采样。
 */
void Board_InitPwmAdcSync(void);

/**
 * @brief 按当前 PWM duty 刷新下一拍电流采样 pair 和 ADC 触发点。
 * @details 由 Board_SetPwmDuty() 在 duty 更新后调用；ADC 同步未启动时该函数不动作。
 */
void Board_UpdateCurrentSampleTiming(void);

/**
 * @brief ADC 中断处理入口。
 * @return 1 表示完成一组电流采样，0 表示无有效采样。
 */
uint8_t Board_AdcIrqHandler(void);

/**
 * @brief Getter: 获取逻辑 U 相电流。
 * @return 零漂补偿后的电流 ADC count。
 */
int16_t Board_GetIuCnt(void);

/**
 * @brief Getter: 获取逻辑 V 相电流。
 * @return 零漂补偿后的电流 ADC count。
 */
int16_t Board_GetIvCnt(void);

/**
 * @brief Getter: 获取逻辑 W 相电流。
 * @return 零漂补偿后的电流 ADC count。
 */
int16_t Board_GetIwCnt(void);

/**
 * @brief Getter: 获取逻辑三相电流和。
 * @return Iu + Iv + Iw，单位 ADC count。
 */
int16_t Board_GetIuvwSum(void);

/**
 * @brief Getter: 获取原始映射 U 通道电流。
 * @return 零漂补偿后的原始通道电流 ADC count。
 */
int16_t Board_GetIuRawCnt(void);

/**
 * @brief Getter: 获取原始映射 V 通道电流。
 * @return 零漂补偿后的原始通道电流 ADC count。
 */
int16_t Board_GetIvRawCnt(void);

/**
 * @brief Getter: 获取原始映射 W 通道电流。
 * @return 零漂补偿后的原始通道电流 ADC count。
 */
int16_t Board_GetIwRawCnt(void);

/**
 * @brief Getter: 获取原始映射三相电流和。
 * @return 原始通道三相和，单位 ADC count。
 */
int16_t Board_GetIuvwRawSum(void);

/**
 * @brief Getter: 获取 raw U 通道 ADC 原始码值。
 * @return 未扣零漂的 ADC 原始值。
 */
uint16_t Board_GetIuRawAdc(void);

/**
 * @brief Getter: 获取 raw V 通道 ADC 原始码值。
 * @return 未扣零漂的 ADC 原始值。
 */
uint16_t Board_GetIvRawAdc(void);

/**
 * @brief Getter: 获取 raw W 通道 ADC 原始码值。
 * @return 未扣零漂的 ADC 原始值。
 */
uint16_t Board_GetIwRawAdc(void);

/**
 * @brief Getter: 获取 ADC/PWM 同步采样计数。
 * @return 成功完成的同步采样次数。
 */
uint32_t Board_GetAdcSyncCount(void);

/**
 * @brief Getter: 判断当前是否启用邻域双点采样。
 * @return 1 表示启用双点采样，0 表示保持单点采样。
 */
uint8_t Board_IsCurrentSampleMultiEnabled(void);

/**
 * @brief Getter: 判断当前是否启用动态低边窗口采样。
 */
uint8_t Board_IsCurrentSampleDynamicEnabled(void);

/**
 * @brief Getter: 获取当前动态采样 pair。
 * @return 0=UV, 1=UW, 2=VW, 255=无有效 pair。
 */
uint8_t Board_GetCurrentSamplePair(void);

/**
 * @brief Getter: 当前周期是否保持上一组有效电流。
 */
uint8_t Board_GetCurrentSampleHold(void);

/**
 * @brief Getter: 动态采样无有效两相窗口的累计次数。
 */
uint16_t Board_GetCurrentSampleHoldCount(void);

/**
 * @brief Getter: 当前动态采样中心 tick。
 */
uint16_t Board_GetCurrentSampleCenterTick(void);

/**
 * @brief Getter: U 相低边窗口宽度。
 */
uint16_t Board_GetCurrentSampleWindowU(void);

/**
 * @brief Getter: V 相低边窗口宽度。
 */
uint16_t Board_GetCurrentSampleWindowV(void);

/**
 * @brief Getter: W 相低边窗口宽度。
 */
uint16_t Board_GetCurrentSampleWindowW(void);

/**
 * @brief Getter: 获取当前采样诊断阶段。
 * @return 0 表示等待第一点，1 表示等待第二点。
 */
uint8_t Board_GetCurrentSampleDiagStage(void);

/**
 * @brief Getter: 获取当前采样诊断累计点数。
 * @return 当前 PWM 周期已收集的有效点数。
 */
uint8_t Board_GetCurrentSampleDiagCount(void);

/**
 * @brief Getter: 获取 A 采样点中当前 pair 的第一相。
 */
int16_t Board_GetCurrentSampleAFirst(void);

/**
 * @brief Getter: 获取 A 采样点中当前 pair 的第二相。
 */
int16_t Board_GetCurrentSampleASecond(void);

/**
 * @brief Getter: 获取 B 采样点中当前 pair 的第一相。
 */
int16_t Board_GetCurrentSampleBFirst(void);

/**
 * @brief Getter: 获取 B 采样点中当前 pair 的第二相。
 */
int16_t Board_GetCurrentSampleBSecond(void);

/**
 * @brief Getter: 获取当前 pair 第一相的 B-A 差值。
 */
int16_t Board_GetCurrentSampleSpreadFirst(void);

/**
 * @brief Getter: 获取当前 pair 第二相的 B-A 差值。
 */
int16_t Board_GetCurrentSampleSpreadSecond(void);

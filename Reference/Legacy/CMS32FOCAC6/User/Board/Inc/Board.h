/**
 * @file Board.h
 * @brief 板级硬件抽象层接口
 * @details 提供板级硬件初始化、ADC 中断处理和传感器读取等功能。
 *          此模块负责管理 CMS32M6513AGE40NB 控制板的所有硬件外设，
 *          包括时钟、GPIO、MA600 SPI、ADC/PGA、EPWM 等。
 */

#pragma once

#include <stdint.h>

/**
 * @brief 板级硬件初始化
 * @details 初始化所有板级硬件外设，包括：
 *          - 系统时钟配置（64 MHz）
 *          - GPIO 引脚配置
 *          - MA600 SPI 接口初始化
 *          - ADC/PGA 电流采样配置
 *          - EPWM PWM 输出配置
 *          - 电流零漂校准
 *          - PWM 触发 ADC 同步采样启动
 * @note 此函数应在系统启动时调用一次
 * @warning 调用前确保电源稳定，否则零漂校准可能不准确
 */
void Board_Init(void);

/**
 * @brief ADC 中断处理函数
 * @details 处理 ADC 转换完成中断，执行以下操作：
 *          1. 读取三相电流采样值（U/V/W）
 *          2. 计算电流零漂补偿后的值
 *          3. 更新 ADC 同步计数
 * @return 非零值表示处理成功，可以执行电机快环；0 表示处理失败
 * @note 此函数在 ADC_IRQHandler() 中调用
 * @warning 此函数在中断上下文中执行，应保持快速返回
 */
uint8_t Board_HandleAdcIrq(void);

/**
 * @brief 更新 MA600 角度缓存
 * @details 执行一次阻塞式 SPI 读角，并把结果保存在 Board_MA600 内部缓存中。
 *          此函数不应在 20 kHz 快环中直接调用，建议在 Motor_TASK 或快环分频路径调用。
 * @return 1 表示本次读角成功，0 表示 SPI 超时或读角失败
 */
uint8_t Board_UpdateAngle(void);

/**
 * @brief 快速更新 MA600 角度缓存（可在中断中调用）
 * @details 使用更短的超时时间，在 ADC 中断中调用以同步角度和电流采样。
 * @return 1 表示本次读角成功，0 表示 SPI 超时或读角失败
 */
uint8_t Board_UpdateAngleFast(void);

/**
 * @brief Getter: 获取 MA600 缓存的原始角度
 * @return 最近一次成功读取的 16 位 MA600 原始角度
 * @note 此函数只读缓存，不产生 SPI 事务，适合快环使用。
 */
uint16_t Board_GetAngleRaw(void);

/**
 * @brief Getter: 获取 MA600 缓存有效标志
 * @return 1 表示最近一次读角成功，0 表示最近一次读角失败
 */
uint8_t Board_IsAngleOk(void);

/**
 * @brief Getter: 获取 MA600 缓存年龄
 * @return 距离最近一次成功更新已经经过的失败次数或老化计数，最大 255
 */
uint8_t Board_GetAngleAge(void);

/**
 * @brief 读取 MA600 角度传感器原始值
 * @details 通过 SPI 接口读取 MA600 磁性角度传感器的原始角度值
 * @return 16 位角度值，范围 0x0000-0xFFFF 对应 0-360 度
 * @retval 0xFFFF 读取失败或传感器异常
 * @note 角度值为磁场角度，不是机械角度
 * @warning 调用前确保 MA600 SPI 已初始化
 */
uint16_t Board_ReadAngleRaw(void);

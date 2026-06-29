//===========================================================================//
/*****************************************************************************/
//---------------------------------------------------------------------------//
/** \file
*  File Name  : Set_FAULT.h
*  Author     : CMS_Motor_Control_Team
*  Version    : 第二代风机软件平台
*  Date       : 09/01/2022
*  Description: 本页参数为电机故障识别、保护相关参数，
								包括但不限于：电压保护、缺相保护等
**             
//---------------------------------------------------------------------------//
******************************************************************************/

//===========================================================================//
#ifndef __SET_PROTECT_H
#define __SET_PROTECT_H


//============================================================================/
// 代码对齐方式：Edit--Configuration--Editor--C/C++Files--Tab size = 2
//----------------------------------------------------------------------------/

//===========================================================================//

// 重启次数设置为72次，可无限次重启

//----------------------------------------------------------------------------/
// 硬件过流(短路保护)
#define		HARDOVCUR_PROTECT_ENABLE							(0)                     // 硬件过流保护 		0:不使能；1:使能
#define		HARDOVCUR_PROTECT_VALUE								(15.0)									// 硬件过流					单位：A */ 

#define		HARDOVCUR_RESTART_ENABLE							(0)                     // 保护后重启使能		0,不使能；1，使能
#define		HARDOVCUR_RESTART_GAP									(5000)                  // 重启间隔时间 		单位：ms
#define		HARDOVCUR_RESTART_NUM									(1)                     // 重启次数     		单位：次

//----------------------------------------------------------------------------/
// 软件过流
#define		OVERPHASECUR_PROTECT_ENABLE						(1)                     // 软件过流保护			0,不使能；1，使能
#define		OVERPHASECUR_PROTECT_VALUE						(8.0)                  // 软件过流保护值		单位：A 
#define		OVERPHASECUR_PROTECT_TIMES						(2)                     // 软件过流次数，连续超过x次保护 单位：Ts（开关周期）  

#define		OVERPHASECUR_RESTART_ENABLE						(0)                     // 保护后重启使能		0,不使能；1，使能
#define		OVERPHASECUR_RESTART_GAP							(10000)                 // 重启间隔时间 		单位：ms
#define		OVERPHASECUR_RESTART_NUM							(2)                     // 重启次数					单位：次

//----------------------------------------------------------------------------/
// 电压保护
#define		VBUS_PROTECT_ENABLE										(1)                     // 电压保护					0,不使能；1，使能
#define		OVER_VOLT_VALUE												(27.0)                  // 过压保护电压值		单位：V
#define		UNDER_VOLT_VALUE											(6.5)                  // 欠压保护电压值		单位：V
#define		VBUS_PROTECT_TIME											(200)                   // 过欠压保护延时		单位：ms

#define		VBUS_RECOVER_ENABLE										(1)                     // 电压保护自行恢复	0,不使能；1，使能
#define		OV_RECOVER_VALUE											(26.0)                  // 过压恢复电压值		单位：V 
#define		UV_RECOVER_VALUE											(11.5)                  // 欠压恢复电压值		单位：V
#define		VBUS_RECOVER_TIME											(1500)                  // 过欠压恢复延时		单位：ms

//----------------------------------------------------------------------------/
// 堵转保护
#define		BLOCK_PROTECT_ENABLE									(0)                     // 堵转保护					0,不使能；1，使能
#define		BLOCK_SPEED_MAX												(5000)                  // 堵转保护最大转速	单位：rpm
#define		BLOCK_SPEED_MIN												(20)                   // 堵转保护最小转速	单位：rpm
#define		BLOCK_PROTECT_TIME										(500)                   // 堵转判别时间			单位：ms

#define		BLOCK_RESTART_ENABLE									(0)                     // 保护后重启使能		0,不使能；1，使能
#define		BLOCK_RESTART_GAP											(5000)                  // 重启间隔时间 		单位：ms
#define		BLOCK_RESTART_NUM											(3)                     // 重启次数					单位：次
								
//----------------------------------------------------------------------------/
// 缺相保护
#define		PHASELOSS_PROTECT_ENABLE							(0)                     // 缺相保护					0,不使能；1，使能
#define		PHASELOSS_CUR_MIN											(0.05)                  // 缺相电流阈值			单位：A
#define		PHASELOSS_PROTECT_TIME								(200)                   // 缺相保护时间

#define		PHASELOSS_RESTART_ENABLE							(0)                     // 保护后重启使能		0,不使能；1，使能
#define		PHASELOSS_RESTART_GAP									(5000)                  // 重启间隔时间			单位：ms
#define		PHASELOSS_RESTART_NUM									(3)                     // 重启次数					单位：次
								
//----------------------------------------------------------------------------/
// 温度保护
#define		TEMP_PROTECT_ENABLE										(0)                     // 温度保护					0,不使能；1，使能
#define		OVER_TEMP_VALUE												(120)                   // 高温保护电压值		单位：℃
#define		UNDER_TEMP_VALUE											(-20)                   // 低温保护电压值		单位：℃
#define		TEMP_PROTECT_TIME											(500)                   // 高低温保护延时		单位：ms

#define		TEMP_RECOVER_ENABLE										(0)                     // 温度保护自行恢复	0,不使能；1，使能
#define		OTEMP_RECOVER_VALUE										(100)                   // 高温恢复温度值		单位：℃
#define		UTEMP_RECOVER_VALUE										(-5)                   	// 低温恢复温度值		单位：℃
#define		TEMP_RECOVER_TIME											(500)                   // 高低温恢复延时		单位：ms
						
//----------------------------------------------------------------------------/
// 启动失败保护
#define		STARTFAIL_PROTECT_ENABLE							(1)                     // 启动保护					0,不使能；1，使能
#define		STARTFAIL_PROTECT_TIME								(5000)                  // 启动失败判别时间	单位：ms

#define		STARTFAIL_RESTART_ENABLE							(1)                     // 保护后重启使能		0,不使能；1，使能
#define		STARTFAIL_RESTART_GAP									(5000)                  // 重启间隔时间			单位：ms
#define		STARTFAIL_RESTART_NUM									(72)                    // 重启次数					单位：次

//----------------------------------------------------------------------------/
// 偏置错误
#define		OFFSET_PROTECT_ENABLE									(0)                     // 偏置错误保护			0,不使能；1，使能
#define		OFFSET_ERROR_VALUE										(0.2)                   // 允许最大偏差电压	单位：V

//----------------------------------------------------------------------------/
// 错误复位设置
#define		FAULTCLEAR_SIGNRESET_EN								(1)                     // 重置控制信号清除所有故障、重启次数	
#define		FAULTCLEAR_RUN_EN											(0)                     // 运行成功后清除重启次数

#endif

/******************************** END OF FILE *******************************/


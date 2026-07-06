/**
 * @file TuneConfig.h
 * @brief 当前调试阶段的采样、保护、控制环和开环默认参数。
 * @details 小电机电流采样仍在调整，优先修改本文件。
 */

#pragma once

/* Current sampling -------------------------------------------------------- */

/** @brief 运行时 CMP0 同步采样 U/V 两相，W 相由 -U-V 重构。 */
#define CS_PAIR_UV 0U
/** @brief 运行时 CMP0 同步采样 U/W 两相，V 相由 -U-W 重构。 */
#define CS_PAIR_UW 1U
/** @brief 运行时 CMP0 同步采样 V/W 两相，U 相由 -V-W 重构。 */
#define CS_PAIR_VW 2U

#define CS_PAIR_SELECT_DYNAMIC 0U
#define CS_PAIR_SELECT_PREFERRED 1U
#define CS_PAIR_SELECT_FIXED 2U
/** @brief 采样 pair 选择模式：固定采样时刻下动态选择当前可采的两相。 */
#define CS_PAIR_SELECT_MODE CS_PAIR_SELECT_DYNAMIC
/** @brief 优先使用的采样 pair；可在 UV/UW/VW 中扫一遍找最干净组合。 */
#define CS_PAIR_PREFERRED CS_PAIR_VW
/** @brief 优先 pair 共同窗口低于该值时，回退动态选择。 */
#define CS_PAIR_PREFERRED_MIN_WINDOW_TICK 500U

/** @brief 同一低边窗口双点采样开关。 */
#define CS_MULTI_EN 1U
/** @brief 双点采样与主采样点的偏移 tick。 */
#define CS_MULTI_DELTA_TICK 4U
/** @brief 双点采样两点差值超过该值时，认为本拍落在开关噪声或恢复区。 */
#define CS_MULTI_SPREAD_LIMIT_CNT 40
/** @brief 双点差值超限时保持上一拍；调试原始波形时关闭。 */
#define CS_MULTI_SPREAD_REJECT_EN 0U

/** @brief 使用固定绝对 EPWM tick 采样；0 表示按 TI T1/T2/T3 镜像窗口动态放 CMP。 */
#define CS_FIXED_SAMPLE_TICK_EN 0U
/** @brief 固定采样中心 tick；FALLING 下应落在下降计数经过 duty 之后。 */
#define CS_FIXED_SAMPLE_TICK 650U

#define CS_LOW_WINDOW_FROM_DUTY 0U
#define CS_LOW_WINDOW_FROM_OFF_TIME 1U
/** @brief 当前先回到已验证过的 duty 窗口模型；动态窗口后续再按扇区重做。 */
#define CS_LOW_WINDOW_MODE CS_LOW_WINDOW_FROM_DUTY
/** @brief 采样点离低边窗口起点的最小恢复时间。 */
#define CS_EDGE_SETTLE_TICK (PWM_DEADTIME_TICKS + 32U)
/** @brief 下降计数经过 duty 后，下管刚打开到 ADC 主采样点的延迟。 */
#define CS_OPEN_SETTLE_TICK (PWM_DEADTIME_TICKS + 4U)//(PWM_DEADTIME_TICKS + 80U)
/** @brief 采样点离低边窗口末端的最小安全余量。 */
#define CS_TAIL_MARGIN_TICK 60U
/** @brief Case3 无效相使用的软件低通滤波右移。 */
#define CS_FILTER_SHIFT 3U
/** @brief T1 只比最小宽度略大时提前让出 case1；当前小电机先尽量保留三相实采窗口。 */
#define CS_REGION1_ENTER_MARGIN_TICK 0U
/** @brief 共同窗口低于该值时不增加中心点偏移。 */
#define CS_BIAS_MID_WINDOW_TICK 500U
/** @brief 共同窗口高于该值时使用较大的中心点偏移。 */
#define CS_BIAS_HIGH_WINDOW_TICK 900U
/** @brief 中等共同窗口的中心点偏移。 */
#define CS_BIAS_MID_TICK 0
/** @brief 大共同窗口的中心点偏移。 */
#define CS_BIAS_HIGH_TICK 0
/** @brief UV pair 的中心点固定偏移。 */
#define CS_PAIR_BIAS_UV_TICK 0
/** @brief UW pair 的中心点固定偏移。 */
#define CS_PAIR_BIAS_UW_TICK 0
/** @brief VW pair 的中心点固定偏移。 */
#define CS_PAIR_BIAS_VW_TICK 0
/** @brief 高 VF 电压时切换为窗口中心单点采样，减少最短低边窗口所需余量。 */
#define CS_HIGH_VF_SINGLE_EN 1U
/** @brief 高 VF 单点采样阈值，单位为 VF 电压命令 count。 */
#define CS_HIGH_VF_SINGLE_VOLTAGE 660
/** @brief 单相低边窗口小于该值时，不参与动态两相选择。 */
#define CS_MIN_WINDOW_TICK (PWM_DEADTIME_TICKS + 32U)
/** @brief 新采样 pair 必须多出的共同窗口 tick，才允许切换。 */
#define CS_PAIR_SWITCH_HYST_TICK 80U
/** @brief 采样 pair 切换前的最小保持 PWM 周期数。 */
#define CS_PAIR_MIN_HOLD_PWM 16U
/** @brief 采样 pair 切换后丢弃的 PWM 周期数。 */
#define CS_PAIR_SWITCH_BLANK_PWM 1U
/** @brief 相邻采样跳变超过该 count 记一次尖峰。 */
#define CS_SPIKE_LIMIT_CNT 100
/** @brief 相邻采样跳变超限时拒收本拍；调试原始波形时关闭。 */
#define CS_SPIKE_REJECT_EN 0U
/** @brief 连续坏样本达到该次数时关闭 PWM；255 表示调试时基本不触发。 */
#define CS_BAD_SAMPLE_SHUTDOWN_COUNT 255U
/** @brief 单个 ADC 扣零漂样本的物理极限；超过说明不是正常 12-bit ADC 电流。 */
#define CS_SAMPLE_ABS_HARD_LIMIT_CNT 8192

/* Motor safety and alignment --------------------------------------------- */

/** @brief 电流采样值检查限值（ADC count）。 */
#define MOT_CHECK_CURR_CNT_LIMIT 32767
/** @brief 三相电流和检查限值（ADC count）。 */
#define MOT_CHECK_SUM_CNT_LIMIT 32767
/** @brief MA600 传感器检查采样次数。 */
#define MOT_CHECK_MA600_SAMPLES 10u
/** @brief MA600 角度缓存允许的最大年龄。 */
#define MOT_ANGLE_MAX_AGE 4u

/** @brief 闭环前对齐 d 轴电压幅值。 */
#define MOT_ALIGN_VD 240
/** @brief 闭环前对齐目标电角度。 */
#define MOT_ALIGN_THETA 0u
/** @brief 闭环前强拖对齐持续 tick 数。 */
#define MOT_ALIGN_TICKS 3000u
/** @brief PWM 运行时零漂重校准采样次数。 */
#define MOT_ALIGN_CAL_SAMPLES 128u
/** @brief 正反拖动零位扫描的 d 轴电压幅值。 */
#define MOT_ALIGN_SCAN_VD 400
/** @brief 正反拖动零位扫描的快速拖动速度给定。 */
#define MOT_ALIGN_SCAN_FAST_SPEED 400L
/** @brief 正反拖动零位扫描的低速采样速度给定。 */
#define MOT_ALIGN_SCAN_SLOW_SPEED 40L
/** @brief 反向快速预拖动半电周期数。 */
#define MOT_ALIGN_SCAN_REV_FAST_HALFCYCLES 20u
/** @brief 正向快速拖动半电周期数。 */
#define MOT_ALIGN_SCAN_FWD_FAST_HALFCYCLES 8u
/** @brief 正/反向低速采样各自持续半电周期数。 */
#define MOT_ALIGN_SCAN_SAMPLE_HALFCYCLES 8u
/** @brief 正反拖动零位扫描最小有效采样数。 */
#define MOT_ALIGN_SCAN_MIN_SAMPLES 128u

/* Current loop ------------------------------------------------------------ */

/** @brief 电流环给定限幅。 */
#define CTRL_CUR_REF_LIMIT 1000
/** @brief 电流环/开环电压限幅，跟随当前 SVPWM duty guard。 */
#define CTRL_CUR_V_LIMIT ((int16_t)PWM_SVPWM_V_LIMIT)
/** @brief 电流环 PI 定点右移位数。 */
#define CTRL_CUR_PI_SHIFT 3u
/** @brief 电流环比例系数。 */
#define CTRL_CUR_KP 4
/** @brief 电流环积分系数。 */
#define CTRL_CUR_KI 1
/** @brief 电流环给定每次更新最大变化 count，用于闭环软启动。 */
#define CTRL_CUR_REF_RAMP_STEP 2
/** @brief 运行电流保护阈值。 */
#define CTRL_CUR_SAFE_LIMIT 400
/** @brief 连续过流计数阈值。 */
#define CTRL_CUR_OVER_LIMIT 4u
/** @brief 快环分频，20 kHz ADC/PWM sync / 2 = about 10 kHz current loop. */
#define CTRL_FAST_LOOP_DIV 2u

/* Speed loop -------------------------------------------------------------- */

/** @brief 速度估算和速度 PI 运行频率。 */
#define CTRL_SPD_EST_HZ 500L
#define CTRL_SPD_FB_SOURCE_DIFF 0U
#define CTRL_SPD_FB_SOURCE_MA600 1U
/** @brief 速度环反馈源：MA600 speed 当前低速尖峰偏大，默认先用角度差分。 */
#define CTRL_SPD_FB_SOURCE CTRL_SPD_FB_SOURCE_DIFF
/** @brief MA600 speed 方向修正；若与差分速度反号，改为 -1。 */
#define CTRL_SPD_MA600_SIGN (1)
/** @brief MA600 speed 输出换算到传感器 raw counts/s 的近似比例，5.722rpm/LSB。 */
#define CTRL_SPD_MA600_COUNTS_PER_SEC_PER_LSB 6251L
/** @brief MA600 speed 单次允许跳变，单位 rpm；超过则认为是低速尖峰。 */
#define CTRL_SPD_MA600_SPIKE_RPM 300
/** @brief MA600 speed 原始低通滤波右移，值越大越平滑。 */
#define CTRL_SPD_MA600_FILTER_SHIFT 5u
/** @brief 速度估算位置死区。 */
#define CTRL_SPD_POS_DEADBAND 16L
/** @brief 速度反馈归零吸附阈值。 */
#define CTRL_SPD_ZERO_SNAP 500L
/** @brief 速度环比例系数，PI 输入为 rpm 误差。 */
#define CTRL_SPD_KP 128
/** @brief 速度环积分系数，初调时先关闭积分，确认方向后再在 Ozone 中加到 1。 */
#define CTRL_SPD_KI 4
/** @brief 速度误差缩放右移位数。 */
#define CTRL_SPD_ERR_SHIFT 6u
/** @brief 速度反馈滤波右移位数。 */
#define CTRL_SPD_FILTER_SHIFT 4u
/** @brief 速度给定小于该值时认为是停机命令，防止零速抖动。 */
#define CTRL_SPD_CMD_DEADBAND 200L
/** @brief 速度给定限幅。 */
#define CTRL_SPD_REF_LIMIT 3000000L
/** @brief 速度环默认 iq 限幅。 */
#define CTRL_SPD_IQ_LIMIT 80

/* Open loop VF/IF --------------------------------------------------------- */

/** @brief VF/IF 开环调试默认速度给定，单位 sensor counts/s，约 36 rpm 机械。 */
#define OL_SPEED_REF 400l
/** @brief VF 开环默认电压幅值。 */
#define OL_VF_VOLTAGE 320
/** @brief IF 开环默认 q 轴电流给定。 */
#define OL_IF_IQ_REF 200
/** @brief IF 开环默认 d 轴电流给定。 */
#define OL_IF_ID_REF 0
/** @brief VF/IF 开环默认超时时间，单位 ms。 */
#define OL_TIMEOUT_MS 30000u
/** @brief 开环速度到电角度步进的定点乘数。 */
#define OL_SPEED_TO_THETA_STEP 131l
/** @brief 开环速度到电角度步进的定点右移位数。 */
#define OL_SPEED_TO_THETA_SHIFT 8u

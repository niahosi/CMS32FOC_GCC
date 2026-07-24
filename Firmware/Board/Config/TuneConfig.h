/**
 * @file TuneConfig.h
 * @brief 已验证的采样、控制环和开环默认参数。
 * @details 当前电流采样采用三电阻低边 T1/T2/T3 动态窗口方案。
 */

#pragma once

/* Current sampling -------------------------------------------------------- */

/** @brief 运行时 CMP0 同步采样 U/V 两相，W 相由 -U-V 重构。 */
#define CS_PAIR_UV 0U
/** @brief 运行时 CMP0 同步采样 U/W 两相，V 相由 -U-W 重构。 */
#define CS_PAIR_UW 1U
/** @brief 运行时 CMP0 同步采样 V/W 两相，U 相由 -V-W 重构。 */
#define CS_PAIR_VW 2U
/** @brief T1 三相均有效窗口也只采两相，降低单 ADC 顺序扫描时差。 */
#define CS_USE_2PHASE_IN_ALL_WINDOW 1U
/** @brief T1 三相均有效窗口默认采样 pair；U/V 的 ADC 顺序时差最短且 pair 稳定。 */
#define CS_ALL_WINDOW_PAIR CS_PAIR_UV

/**
 * @brief 同一低边窗口的采样次数：0 表示只在动态窗口中心采一次。
 *
 * 保留 T1/T2/T3 分区、死区后延迟、尾部余量、两相采样和 KCL 重构；
 * 仅关闭 CMP1 的第二次触发与 A/B 平均，让每个 PWM 周期形成一个控制样本。
 */
#define CS_MULTI_EN 0U
/** @brief 双点模式备用的 A/B 偏移 tick；CS_MULTI_EN=0 时不参与运行。 */
#define CS_MULTI_DELTA_TICK 4U
/** @brief 双点模式备用的 A/B 差值阈值；CS_MULTI_EN=0 时不参与运行。 */
#define CS_MULTI_SPREAD_LIMIT_CNT 40
/** @brief 下降计数经过 duty 后，下管刚打开到 ADC 主采样点的延迟。 */
#define CS_OPEN_SETTLE_TICK (PWM_DEADTIME_TICKS + 4U) //(PWM_DEADTIME_TICKS + 80U)
/** @brief 采样点离低边窗口末端的最小安全余量。 */
#define CS_TAIL_MARGIN_TICK 60U
/** @brief Case3 无效相使用的软件低通滤波右移。 */
#define CS_FILTER_SHIFT 3U
/** @brief 双点模式下高 VF 电压切换为中心单点；CS_MULTI_EN=0 时恒为单点。 */
#define CS_HIGH_VF_SINGLE_EN 1U
/** @brief 高 VF 单点采样阈值，单位为 VF 电压命令 count。 */
#define CS_HIGH_VF_SINGLE_VOLTAGE 660
/** @brief 采样 pair 切换后丢弃的 PWM 周期数。 */
#define CS_PAIR_SWITCH_BLANK_PWM 1U
/** @brief 单个 ADC 扣零漂样本的物理极限；超过说明不是正常 12-bit ADC 电流。 */
#define CS_SAMPLE_ABS_HARD_LIMIT_CNT 8192

/* Motor safety and alignment --------------------------------------------- */

/** @brief 电流采样值检查限值（ADC count）。 */
#define MOT_CHECK_CURR_CNT_LIMIT 32767
/** @brief 三相电流和检查限值（ADC count）。 */
#define MOT_CHECK_SUM_CNT_LIMIT 32767
/** @brief MA600 传感器检查采样次数。 */
#define MOT_CHECK_MA600_SAMPLES 10u
/**
 * @brief MA600 角度缓存允许的最大年龄，单位为 Current/Speed 快环拍数。
 *
 * 20 kHz 电流环下 20 拍约 1 ms。高 iq 调试时 PWM/电流扰动可能造成连续
 * 数帧 SPI 坏角，先允许短时保持上一角度；超过该窗口仍判编码器故障。
 */
#define MOT_ANGLE_MAX_AGE 20u
/**
 * @brief 单次控制周期允许的最大 MA600 raw 跳变；超过认为是 SPI/角度坏样本。
 *
 * 首帧角度会无条件接受；进入运行后该门限作为 20 kHz 位置累计的最小 raw
 * 步进门限，用于拒绝 SPI/角度毛刺。5000 rpm 时每拍约 1092 count，
 * 2048 仍有裕量，同时能挡掉明显不可能的单拍跳变。
 */
#define MOT_ENCODER_MAX_STEP_RAW 2048U
/** @brief MA600 侧轴 BCT 补偿是否启用；只写 RAM 寄存器，不存 NVM。 */
#define MOT_ENCODER_SIDE_BCT_EN 1U
/** @brief MA600 侧轴 BCT 强度，按手册 BCT = 258 * (1 - 1 / k)。 */
#define MOT_ENCODER_SIDE_BCT 180U
/** @brief MA600 是否削弱 X 轴；削弱磁场幅值较大的轴。 */
#define MOT_ENCODER_SIDE_ETX 0U
/** @brief MA600 是否削弱 Y 轴；ETX/ETY 不要同时为 1。 */
#define MOT_ENCODER_SIDE_ETY 1U

#if (MOT_ENCODER_SIDE_BCT > 255U)
#error "MOT_ENCODER_SIDE_BCT must be 0..255"
#endif

#if (MOT_ENCODER_SIDE_ETX > 1U) || (MOT_ENCODER_SIDE_ETY > 1U)
#error "MOT_ENCODER_SIDE_ETX and MOT_ENCODER_SIDE_ETY must be 0 or 1"
#endif

#if (MOT_ENCODER_SIDE_ETX != 0U) && (MOT_ENCODER_SIDE_ETY != 0U)
#error "MA600 BCT should trim only one axis; do not enable ETX and ETY together"
#endif

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
#define MOT_ALIGN_SCAN_FAST_SPEED 100L
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
/** @brief 快环分频，20 kHz ADC/PWM sync / 1 = about 20 kHz current loop. */
#define CTRL_FAST_LOOP_DIV 1u

/* Speed loop -------------------------------------------------------------- */

/** @brief 速度估算和速度 PI 运行频率。 */
#define CTRL_SPD_EST_HZ 1000L
/** @brief 模式切入后跳过的速度估算窗口数，避免初始 raw/prev raw 不连续导致速度尖峰。 */
#define CTRL_SPD_STARTUP_BLANK_SAMPLES 4U

/**
 * @brief 角度差分测速的单次跳变拒绝阈值，单位机械 rpm。
 *
 * 这里保护的是 1 kHz 速度采样间隔内的 raw 差分，不是 20 kHz 快环单拍角度
 * 门限。5000 rpm 下每个 1 ms 速度样本约 21845 raw count；6500 rpm 仍低于
 * int16 raw 差分半圈限制，可以放过正常高速，同时拒绝接近半圈的毛刺。
 */
#define CTRL_SPD_DIFF_SPIKE_RPM 6500L
/** @brief 速度估算位置死区。 */
#define CTRL_SPD_POS_DEADBAND 16L
/** @brief 速度反馈归零吸附阈值。 */
#define CTRL_SPD_ZERO_SNAP 500L
/** @brief 速度环比例系数，PI 输入为 rpm 误差；32/1024 = 0.03125 iq/rpm。 */
#define CTRL_SPD_KP 32
/** @brief 速度环积分系数，用于消除稳定速度误差；3/1024 iq/rpm/sample。 */
#define CTRL_SPD_KI 3
/** @brief 速度误差缩放右移位数。 */
#define CTRL_SPD_ERR_SHIFT 10u
/** @brief 速度反馈滤波右移位数。 */
#define CTRL_SPD_FILTER_SHIFT 2u
/** @brief 速度给定小于该 rpm 时认为是停机命令，防止零速抖动。 */
#define CTRL_SPD_CMD_DEADBAND_RPM 5
/**
 * @brief 速度目标斜坡，单位 rpm/s。
 *
 * 5000 rpm 仅需 50 ms 即可到达给定。位置模式的减速不依赖此处的对称斜坡，
 * 而由 CTRL_POS_BRAKE_ACCEL_RPM_PER_S 的距离制动包络提前限制。
 */
#define CTRL_SPD_REF_RAMP_RPM_PER_S 100000L
/** @brief 速度给定限幅，机械 rpm；Ozone speed_ref_rpm 会先换算再按该值限幅。 */
#define CTRL_SPD_REF_LIMIT_RPM 5600L
/** @brief 速度给定限幅，编码器电角 count/s。 */
#define CTRL_SPD_REF_LIMIT                                                             \
    ((CTRL_SPD_REF_LIMIT_RPM * (long)MOT_SENSOR_CPR * (long)MOT_POLE_PAIRS) / 60L)
/** @brief 速度环默认 iq 限幅；80 count 约 0.44 A，4 ohm 电机上压降约 1.76 V。 */
#define CTRL_SPD_IQ_LIMIT 80
/** @brief 速度环输出 iq 命令每个 1 kHz 速度周期允许变化的最大 count。 */
#define CTRL_SPD_IQ_SLEW_STEP 4

/* Position loop ----------------------------------------------------------- */

/** @brief 位置环比例系数；输入为 encoder count 误差，输出为 mechanical rpm。 */
#define CTRL_POS_KP 3
/** @brief 位置环误差缩放右移位数；默认 3/256 rpm/count，0.50 mm 误差会打满 3000 rpm。
 */
#define CTRL_POS_ERR_SHIFT 8u
/**
 * @brief 位置模式默认速度限幅，mechanical rpm。
 *
 * 0.50 mm/rev 丝杠下 5000 rpm 约为 41.7 mm/s。速度环总上限保留在 5600 rpm，
 * 位置环接近目标时由制动规划提前降低速度，避免以机械端点制动。
 */
#define CTRL_POS_SPEED_LIMIT_RPM 5000
/**
 * @brief 位置模式默认 q 轴电流限幅。
 *
 * 120 count 约 0.66 A。位置大阶跃需要同时使用正向加速和负向制动转矩，
 * 因此独立于普通速度模式的 CTRL_SPD_IQ_LIMIT = 80。
 */
#define CTRL_POS_IQ_LIMIT 120
/**
 * @brief 位置环制动规划加速度，单位 mechanical rpm/s。
 *
 * 位置环根据剩余 encoder count 计算可在此加速度内刹停的最大速度，避免纯 P 输出
 * 长时间顶住速度限幅后才开始减速。该值必须小于 CTRL_SPD_REF_RAMP_RPM_PER_S，
 * 为速度反馈滤波、速度 PI 和实际负载变化留出制动余量。
 */
#define CTRL_POS_BRAKE_ACCEL_RPM_PER_S 35000L
/** @brief 位置误差小于该 count 时认为到位，并给速度环 0 rpm。 */
#define CTRL_POS_DEADBAND_COUNTS 128L

/* Open loop VF/IF --------------------------------------------------------- */

/** @brief VF/IF 开环调试默认速度给定，单位 sensor counts/s。 */
#define OL_SPEED_REF 50l
/** @brief VF 开环默认电压幅值。 */
#define OL_VF_VOLTAGE 80
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

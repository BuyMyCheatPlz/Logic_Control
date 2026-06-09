/* Project tunable configuration macros
 * Central file for parameters you may want to tweak during tuning.
 */
#ifndef __PROJECT_CONFIG_H
#define __PROJECT_CONFIG_H

/* Physical and encoder constants */
#define GRID_SIZE_M 0.15f    /* 单格长度, unit: m */
#define WHEEL_DIAM_M 0.06f   /* 轮子直径, unit: m */
#define WHEEL_BASE_M 0.1226f /* 前后轮中心距, unit: m */
#define WHEEL_TRACK_M 0.175f /* 左右轮中心距, unit: m */
#define ENCODER_LINES 13     /* 编码器每转线数, unit: line/rev */
#define ENCODER_QUADRATURE 4 /* 编码器四倍频计数, unit: counts/line */
#define GEAR_RATIO 20        /* 减速比, unit: ratio */

/* Default movement percentages (used by legacy commands) */
#define DEFAULT_STRAFE_PERCENT 30.0f  /* 默认横移速度, unit: % */
/* 前/后退实际距离修正因子。因机械摩擦/打滑，实际行走距离偏短时增大此值。
 * 例：命令 30cm 实测 28.75cm -> 30/28.75 ≈ 1.0435f */
#define FORWARD_CORRECTION_FACTOR 1.0435f /* 前进/后退编码器计数补偿因子, unit: ratio */
/* 左/右横移实际距离修正因子。因机械摩擦/打滑，实际行走距离偏短时增大此值。
 * 例：命令 60cm 实测 62.5cm -> 60/62.5 = 0.96 -> 1.11f * 0.96 ≈ 1.0656f */
#define STRAFE_CORRECTION_FACTOR 1.0656f /* 左/右横移编码器计数补偿因子, unit: ratio */

/* Approximate wheel travel required for one in-place 360-degree turn.
 * Based on the mecanum chassis geometry; adjust WHEEL_BASE_M / WHEEL_TRACK_M
 * in this file to calibrate the turn distance for your robot.
 */
#define CIRCLE_TURN_WHEEL_TRAVEL_M (3.14159265359f * (WHEEL_BASE_M + WHEEL_TRACK_M)) /* 原地转一圈时单轮估算行程, unit: m */
/* 原地旋转实际角度修正因子。因机械摩擦/打滑，实际旋转角度偏小时增大此值。
 * 例：命令 0.25圈(90°) 实测 86° -> 90/86 ≈ 1.0465, 0.508*1.0465 ≈ 0.5316f */
#define TURN_CORRECTION_FACTOR 0.5316f /* CIRCLE旋转编码器计数补偿因子, unit: ratio */
/* LEFT/RIGHT命令中的90° quarter-turn使用独立修正因子，与CIRCLE分离调校 */
#define QUARTER_TURN_CORRECTION_FACTOR 0.5316f /* quarter-turn编码器计数补偿因子, unit: ratio */

/* Command-level output limit: any high-level command (FORWARD/BACKWARD/LEFT/RIGHT/RUN
 * that sets motor percent or starts position motion) will be capped to this percent
 * of full PWM output (0.0f - 100.0f). Set to 30.0f to limit to 30%.
 */
#define COMMAND_MAX_OUTPUT_PERCENT 30.0f /* 高层命令最大输出限幅, unit: % */

/* Position move control */
#define COMMAND_MOTION_TIMEOUT_S 10U                           /* 非circle位置运动超时时间, unit: s */
#define POSITION_TOLERANCE_COUNTS 20                           /* 位置到达容差, unit: counts */
#define POSITION_TIMEOUT_MS (COMMAND_MOTION_TIMEOUT_S * 1000U) /* 非circle位置运动超时时间, unit: ms */
#define CIRCLE_MOTION_TIMEOUT_S 30U                            /* circle位置运动超时时间, unit: s */
#define CIRCLE_TIMEOUT_MS (CIRCLE_MOTION_TIMEOUT_S * 1000U)    /* circle位置运动超时时间, unit: ms */

/* Control-loop low-pass filter */
/* 0.0f -> no update, 1.0f -> no smoothing. */
#define MOTOR_FEEDBACK_LPF_ALPHA 0.20f /* 电机反馈低通系数, unit: 0-1 */

/* Motor / control limits */
#define MOTOR_PWM_PERIOD 999.0f /* PWM 计数周期上限, unit: ticks */
/* Estimated max speed in encoder counts/second (adjust to your hardware) */
#define MOTOR_MAX_SPEED_COUNTS_PER_SEC 3000.0f /* 估计最大轮速, unit: counts/s */

/* Speed deadband: when the absolute effective target speed (counts/s) is below
 * this threshold, PWM output is forced to 0 so the wheels stop. This applies to
 * all motion commands (RUN, FORWARD, BACKWARD, LEFT, RIGHT, CIRCLE, mecanum).
 * Increase if motors still creep at very low speed targets. Unit: counts/s */
#define SPEED_THRESHOLD_COUNTS_PER_SEC 50.0f   /* 速度死区阈值, unit: counts/s */

/* Velocity (inner) PID default gains per wheel.
 * Use explicit macros per wheel so you can tune each independently.
 * Wheel naming: RIGHT_REAR (RR), RIGHT_FRONT (RF), LEFT_FRONT (LF), LEFT_REAR (LR)
 */
/* KP */
#define VELOCITY_PID_KP_RR 2.3f /* 右后轮速度环 Kp, unit: output/(counts/s) */
#define VELOCITY_PID_KP_RF 2.3f /* 左后轮速度环 Kp, unit: output/(counts/s) */
#define VELOCITY_PID_KP_LF 2.3f /* 右前轮速度环 Kp, unit: output/(counts/s) */
#define VELOCITY_PID_KP_LR 2.3f /* 左前轮速度环 Kp, unit: output/(counts/s) */
/* KI */
#define VELOCITY_PID_KI_RR 0.8f /* 右后轮速度环 Ki, unit: output/(counts) */
#define VELOCITY_PID_KI_RF 0.8f /* 左前轮速度环 Ki, unit: output/(counts) */
#define VELOCITY_PID_KI_LF 0.8f /* 左前轮速度环 Ki, unit: output/(counts) */
#define VELOCITY_PID_KI_LR 0.8f /* 左后轮速度环 Ki, unit: output/(counts) */
/* KD */
#define VELOCITY_PID_KD_RR 0.035f  /* 右后轮速度环 Kd, unit: output/(counts/s) */
#define VELOCITY_PID_KD_RF 0.032f  /* 右前轮速度环 Kd, unit: output/(counts/s) */
#define VELOCITY_PID_KD_LF 0.0325f /* 左前轮速度环 Kd, unit: output/(counts/s) */
#define VELOCITY_PID_KD_LR 0.032f  /* 左后轮速度环 Kd, unit: output/(counts/s) */

/* Position (outer) PID default gains */
#define POSITION_PID_KP 1.3f  /* 位置环 Kp, unit: counts/s per count */
#define POSITION_PID_KI 0.07f /* 位置环 Ki, unit: counts/s per (count*s) */
#define POSITION_PID_KD 0.0f  /* 位置环 Kd, unit: counts/s per (count/s) */
/* Limit the position PID output as a percentage of max motor speed.
 * e.g. 100.0f = full speed (MOTOR_MAX_SPEED_COUNTS_PER_SEC counts/s).
 * Per-wheel macros so you can tune each independently. */
#define POSITION_OUTPUT_LIMIT_RR 30.0f /* 右后轮位置环输出限幅, unit: % of max speed */
#define POSITION_OUTPUT_LIMIT_RL 30.0f /* 左后轮位置环输出限幅, unit: % of max speed */
#define POSITION_OUTPUT_LIMIT_FR 30.0f /* 右前轮位置环输出限幅, unit: % of max speed */
#define POSITION_OUTPUT_LIMIT_FL 30.0f /* 左前轮位置环输出限幅, unit: % of max speed */
/* Per-wheel deadzone compensation: extra PWM duty added in the same direction
 * as the commanded output, to overcome static friction per wheel.
 * Unit: percent of full PWM duty (0.0f - 100.0f). 0.0f = no extra compensation.
 * Example: MOTOR_DEADZONE_COMPENSATION_LF 2.0f means +2% extra duty when that wheel is active. */
#define MOTOR_DEADZONE_COMPENSATION_RR 0.0f
#define MOTOR_DEADZONE_COMPENSATION_LR 1.0f
#define MOTOR_DEADZONE_COMPENSATION_RF 0.0f
#define MOTOR_DEADZONE_COMPENSATION_LF 2.0f /**/

#endif /* __PROJECT_CONFIG_H */
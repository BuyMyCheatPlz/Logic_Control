/* Project tunable configuration macros
 * Central file for parameters you may want to tweak during tuning.
 */
#ifndef __PROJECT_CONFIG_H
#define __PROJECT_CONFIG_H

/* Physical and encoder constants */
#define GRID_SIZE_M                 0.30f   /* 单格长度, unit: m */
#define WHEEL_DIAM_M                0.06f   /* 轮子直径, unit: m */
#define WHEEL_BASE_M                0.24f   /* 前后轮中心距, unit: m */
#define WHEEL_TRACK_M               0.20f   /* 左右轮中心距, unit: m */
#define ENCODER_LINES               13      /* 编码器每转线数, unit: line/rev */
#define ENCODER_QUADRATURE         4      /* 编码器四倍频计数, unit: counts/line */
#define GEAR_RATIO                 20      /* 减速比, unit: ratio */

/* Default movement percentages (used by legacy commands) */
#define DEFAULT_STRAFE_PERCENT     30.0f   /* 默认横移速度, unit: % */
#define DEFAULT_FORWARD_PERCENT    30.0f   /* 默认前进速度, unit: % */

/* Approximate wheel travel required for one in-place 360-degree turn.
 * Based on the mecanum chassis geometry; adjust WHEEL_BASE_M / WHEEL_TRACK_M
 * in this file to calibrate the turn distance for your robot.
 */
#define CIRCLE_TURN_WHEEL_TRAVEL_M (3.14159265359f * (WHEEL_BASE_M + WHEEL_TRACK_M)) /* 原地转一圈时单轮估算行程, unit: m */

/* Command-level output limit: any high-level command (FORWARD/BACKWARD/LEFT/RIGHT/RUN
 * that sets motor percent or starts position motion) will be capped to this percent
 * of full PWM output (0.0f - 100.0f). Set to 30.0f to limit to 30%.
 */
#define COMMAND_MAX_OUTPUT_PERCENT 30.0f   /* 高层命令最大输出限幅, unit: % */

/* Position move control */
#define COMMAND_MOTION_TIMEOUT_S   10U     /* 位置运动超时时间, unit: s */
#define POSITION_TOLERANCE_COUNTS  10      /* 位置到达容差, unit: counts */
#define POSITION_TIMEOUT_MS        (COMMAND_MOTION_TIMEOUT_S * 1000U) /* 位置运动超时时间, unit: ms */
#define POSITION_POLL_DELAY_MS     10U     /* 位置轮询周期, unit: ms */

/* VOFA JustFloat telemetry */
#define VOFA_JUSTFLOAT_PERIOD_MS   20U     /* VOFA 发送周期, unit: ms */
#define VOFA_JUSTFLOAT_FLOATS      8U      /* VOFA 每帧浮点数个数, unit: count */

/* Control-loop low-pass filters */
/* 0.0f -> no update, 1.0f -> no smoothing. */
#define MOTOR_FEEDBACK_LPF_ALPHA   0.20f   /* 电机反馈低通系数, unit: 0-1 */
#define HEADING_PITCH_LPF_ALPHA    0.20f   /* 航向/姿态低通系数, unit: 0-1 */

/* Motor / control limits */
#define MOTOR_PWM_PERIOD          999.0f   /* PWM 计数周期上限, unit: ticks */
/* Estimated max speed in encoder counts/second (adjust to your hardware) */
#define MOTOR_MAX_SPEED_COUNTS_PER_SEC 3000.0f /* 估计最大轮速, unit: counts/s */

/* Velocity (inner) PID default gains per wheel.
 * Use explicit macros per wheel so you can tune each independently.
 * Wheel naming: RIGHT_REAR (RR), RIGHT_FRONT (RF), LEFT_FRONT (LF), LEFT_REAR (LR)
 */
/* KP */
#define VELOCITY_PID_KP_RR         1.0f    /* 右后轮速度环 Kp, unit: output/(counts/s) */
#define VELOCITY_PID_KP_RF         0.8f    /* 右前轮速度环 Kp, unit: output/(counts/s) */
#define VELOCITY_PID_KP_LF         0.5f    /* 左前轮速度环 Kp, unit: output/(counts/s) */
#define VELOCITY_PID_KP_LR         0.5f    /* 左后轮速度环 Kp, unit: output/(counts/s) */
/* KI */
#define VELOCITY_PID_KI_RR         0.4f    /* 右后轮速度环 Ki, unit: output/(counts) */
#define VELOCITY_PID_KI_RF         0.05f   /* 右前轮速度环 Ki, unit: output/(counts) */
#define VELOCITY_PID_KI_LF         0.05f   /* 左前轮速度环 Ki, unit: output/(counts) */
#define VELOCITY_PID_KI_LR         0.05f   /* 左后轮速度环 Ki, unit: output/(counts) */
/* KD */
#define VELOCITY_PID_KD_RR         0.05f   /* 右后轮速度环 Kd, unit: output/(counts/s) */
#define VELOCITY_PID_KD_RF         0.0f    /* 右前轮速度环 Kd, unit: output/(counts/s) */
#define VELOCITY_PID_KD_LF         0.0f    /* 左前轮速度环 Kd, unit: output/(counts/s) */
#define VELOCITY_PID_KD_LR         0.0f    /* 左后轮速度环 Kd, unit: output/(counts/s) */

/* Position (outer) PID default gains */
#define POSITION_PID_KP            0.5f     /* 位置环 Kp, unit: counts/s per count */
#define POSITION_PID_KI            0.0f     /* 位置环 Ki, unit: counts/s per (count*s) */
#define POSITION_PID_KD            0.0f     /* 位置环 Kd, unit: counts/s per (count/s) */
/* Limit the position PID output (counts/sec) to this value */
#define POSITION_OUTPUT_LIMIT      (MOTOR_MAX_SPEED_COUNTS_PER_SEC) /* 位置环输出限幅, unit: counts/s */
/* Tolerance (counts) used to determine target reached */
#define POSITION_TOLERANCE_COUNTS   10      /* 位置到达容差, unit: counts */

/* Heading PID removed */

#endif /* __PROJECT_CONFIG_H */

/* Project tunable configuration macros
 * Central file for parameters you may want to tweak during tuning.
 */
#ifndef __PROJECT_CONFIG_H
#define __PROJECT_CONFIG_H

/* Physical and encoder constants */
#define GRID_SIZE_M                 0.30f
#define WHEEL_DIAM_M                0.06f
#define ENCODER_LINES               13
#define ENCODER_QUADRATURE         4
#define GEAR_RATIO                 20

/* Default movement percentages (used by legacy commands) */
#define DEFAULT_STRAFE_PERCENT     30.0f
#define DEFAULT_FORWARD_PERCENT    30.0f

/* Position move control */
#define POSITION_TOLERANCE_COUNTS  10
#define POSITION_TIMEOUT_MS        10000U
#define POSITION_POLL_DELAY_MS     10U

/* VOFA JustFloat telemetry */
#define VOFA_JUSTFLOAT_PERIOD_MS   20U
#define VOFA_JUSTFLOAT_FLOATS      8U

/* Motor / control limits */
#define MOTOR_PWM_PERIOD          999.0f
/* Estimated max speed in encoder counts/second (adjust to your hardware) */
#define MOTOR_MAX_SPEED_COUNTS_PER_SEC 3000.0f

/* Velocity (inner) PID default gains per wheel.
 * Use explicit macros per wheel so you can tune each independently.
 * Wheel naming: RIGHT_REAR (RR), RIGHT_FRONT (RF), LEFT_FRONT (LF), LEFT_REAR (LR)
 */
/* KP */
#define VELOCITY_PID_KP_RR         0.5f
#define VELOCITY_PID_KP_RF         0.5f
#define VELOCITY_PID_KP_LF         0.5f
#define VELOCITY_PID_KP_LR         0.5f
/* KI */
#define VELOCITY_PID_KI_RR         0.05f
#define VELOCITY_PID_KI_RF         0.05f
#define VELOCITY_PID_KI_LF         0.05f
#define VELOCITY_PID_KI_LR         0.05f
/* KD */
#define VELOCITY_PID_KD_RR         0.0f
#define VELOCITY_PID_KD_RF         0.0f
#define VELOCITY_PID_KD_LF         0.0f
#define VELOCITY_PID_KD_LR         0.0f

/* Position (outer) PID default gains */
#define POSITION_PID_KP            0.0f
#define POSITION_PID_KI            0.0f
#define POSITION_PID_KD            0.0f
/* Limit the position PID output (counts/sec) to this value */
#define POSITION_OUTPUT_LIMIT      (MOTOR_MAX_SPEED_COUNTS_PER_SEC)
/* Tolerance (counts) used to determine target reached */
#define POSITION_TOLERANCE_COUNTS   10

/* Heading PID defaults */
#define HEADING_PID_KP             0.0f
#define HEADING_PID_KI             0.0f
#define HEADING_PID_KD             0.0f
#define HEADING_PID_OUTPUT_LIMIT   20.0f

#endif /* __PROJECT_CONFIG_H */

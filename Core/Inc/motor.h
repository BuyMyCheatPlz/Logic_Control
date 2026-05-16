#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "config.h"

typedef enum
{
	MOTOR_RIGHT_REAR = 0,
	MOTOR_LEFT_REAR,
	MOTOR_RIGHT_FRONT,
	MOTOR_LEFT_FRONT,
	MOTOR_COUNT
} MotorId_t;

typedef struct
{
	float kp;
	float ki;
	float kd;
	float target;
	float feedback;
	float error;
	float integral;
	float last_error;
	float output;
	float output_min;
	float output_max;
} MotorPid_t;

void Motor_Init(void);
void Motor_StopAll(void);
void Motor_SetInversion(MotorId_t motor, uint8_t inverted);
void Motor_SetPIDGain(MotorId_t motor, float kp, float ki, float kd);
void Motor_SetPositionPIDGain(MotorId_t motor, float kp, float ki, float kd);
void Motor_SetPositionTarget(MotorId_t motor, int32_t counts_relative);
void Motor_ClearPositionTarget(MotorId_t motor);
int32_t Motor_GetPosition(MotorId_t motor);
uint8_t Motor_PositionReached(MotorId_t motor, int32_t tolerance);
uint8_t Motor_HasActivePositionTarget(void);
void Motor_SetTarget(MotorId_t motor, float target);
void Motor_SetTargetPercent(MotorId_t motor, float percent);
void Motor_SetTargetBiasPercent(MotorId_t motor, float percent_bias);
void Motor_ClearTargetBias(MotorId_t motor);
void Motor_SetOutputLimit(MotorId_t motor, float min_output, float max_output);
void Motor_ResetPID(MotorId_t motor);
void Motor_UpdateControl(float dt_s);
void Motor_SetRawPWM(MotorId_t motor, int32_t pwm);
void Motor_SetEncoderInversion(MotorId_t motor, uint8_t inverted);
int32_t Motor_GetEncoderDelta(MotorId_t motor);
float Motor_GetFeedback(MotorId_t motor);
const MotorPid_t *Motor_GetPID(MotorId_t motor);
void Motor_PositionUpdate(float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */

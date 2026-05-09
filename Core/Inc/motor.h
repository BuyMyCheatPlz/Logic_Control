#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

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
void Motor_SetTarget(MotorId_t motor, float target);
void Motor_SetTargetPercent(MotorId_t motor, float percent);
void Motor_SetOutputLimit(MotorId_t motor, float min_output, float max_output);
void Motor_ResetPID(MotorId_t motor);
void Motor_UpdateControl(float dt_s);
void Motor_SetRawPWM(MotorId_t motor, int32_t pwm);
int32_t Motor_GetEncoderDelta(MotorId_t motor);
float Motor_GetFeedback(MotorId_t motor);
const MotorPid_t *Motor_GetPID(MotorId_t motor);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */

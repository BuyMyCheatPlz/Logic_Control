/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : motor.c
	* @brief          : Four-wheel TB6612 motor driver with per-wheel PID.
	******************************************************************************
	*/
/* USER CODE END Header */

#include "motor.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

typedef struct
{
	TIM_HandleTypeDef *pwm_timer;
	uint32_t pwm_channel;
	TIM_HandleTypeDef *encoder_timer;
	GPIO_TypeDef *in1_port;
	uint16_t in1_pin;
	GPIO_TypeDef *in2_port;
	uint16_t in2_pin;
} MotorHardware_t;

typedef struct
{
	MotorPid_t pid;
	uint8_t inverted;
	uint32_t last_count;
} MotorState_t;

#define MOTOR_PWM_PERIOD 999.0f
#define MOTOR_MAX_SPEED_COUNTS_PER_SEC 3000.0f

static const MotorHardware_t motor_hw[MOTOR_COUNT] =
{
	{&htim1, TIM_CHANNEL_1, &htim5, GPIOE, GPIO_PIN_7,  GPIOE, GPIO_PIN_8},
	{&htim1, TIM_CHANNEL_2, &htim2, GPIOA, GPIO_PIN_9,  GPIOA, GPIO_PIN_8},
	{&htim1, TIM_CHANNEL_3, &htim3, GPIOE, GPIO_PIN_12, GPIOE, GPIO_PIN_10},
	{&htim1, TIM_CHANNEL_4, &htim4, GPIOC, GPIO_PIN_6,  GPIOC, GPIO_PIN_8},
};

static MotorState_t motor_state[MOTOR_COUNT];

static int32_t motor_clamp_int32(int32_t value, int32_t min_value, int32_t max_value)
{
	if (value < min_value)
	{
		return min_value;
	}
	if (value > max_value)
	{
		return max_value;
	}
	return value;
}

static float motor_clamp_float(float value, float min_value, float max_value)
{
	if (value < min_value)
	{
		return min_value;
	}
	if (value > max_value)
	{
		return max_value;
	}
	return value;
}

static void motor_apply_direction(MotorId_t motor, uint8_t forward)
{
	uint8_t output_high = forward ? GPIO_PIN_SET : GPIO_PIN_RESET;
	uint8_t output_low = forward ? GPIO_PIN_RESET : GPIO_PIN_SET;

	if (motor_state[motor].inverted)
	{
		output_high = forward ? GPIO_PIN_RESET : GPIO_PIN_SET;
		output_low = forward ? GPIO_PIN_SET : GPIO_PIN_RESET;
	}

	HAL_GPIO_WritePin(motor_hw[motor].in1_port, motor_hw[motor].in1_pin, output_high);
	HAL_GPIO_WritePin(motor_hw[motor].in2_port, motor_hw[motor].in2_pin, output_low);
}

static int32_t motor_read_delta(MotorId_t motor)
{
	uint32_t current = __HAL_TIM_GET_COUNTER(motor_hw[motor].encoder_timer);
	uint32_t last = motor_state[motor].last_count;
	uint32_t raw_diff = current - last;
	int32_t signed_diff;

	if ((motor_hw[motor].encoder_timer->Instance == TIM2) || (motor_hw[motor].encoder_timer->Instance == TIM5))
	{
		if (raw_diff > 0x7FFFFFFFU)
		{
			signed_diff = (int32_t)(raw_diff - 0x100000000ULL);
		}
		else
		{
			signed_diff = (int32_t)raw_diff;
		}
	}
	else
	{
		raw_diff &= 0xFFFFU;
		if (raw_diff > 0x7FFFU)
		{
			signed_diff = (int32_t)(raw_diff - 0x10000U);
		}
		else
		{
			signed_diff = (int32_t)raw_diff;
		}
	}

	if (motor_state[motor].inverted)
	{
		signed_diff = -signed_diff;
	}

	motor_state[motor].last_count = current;
	motor_state[motor].pid.feedback = (float)signed_diff;

	return signed_diff;
}

static MotorId_t motor_valid_id(MotorId_t motor)
{
	if ((motor < MOTOR_RIGHT_REAR) || (motor >= MOTOR_COUNT))
	{
		return MOTOR_RIGHT_REAR;
	}

	return motor;
}

void Motor_Init(void)
{
	for (MotorId_t motor = MOTOR_RIGHT_REAR; motor < MOTOR_COUNT; motor++)
	{
		motor_state[motor].inverted = 0;
		motor_state[motor].last_count = 0;
		motor_state[motor].pid.kp = 0.0f;
		motor_state[motor].pid.ki = 0.0f;
		motor_state[motor].pid.kd = 0.0f;
		motor_state[motor].pid.target = 0.0f;
		motor_state[motor].pid.feedback = 0.0f;
		motor_state[motor].pid.error = 0.0f;
		motor_state[motor].pid.integral = 0.0f;
		motor_state[motor].pid.last_error = 0.0f;
		motor_state[motor].pid.output = 0.0f;
		motor_state[motor].pid.output_min = -MOTOR_PWM_PERIOD;
		motor_state[motor].pid.output_max = MOTOR_PWM_PERIOD;

		HAL_GPIO_WritePin(motor_hw[motor].in1_port, motor_hw[motor].in1_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor_hw[motor].in2_port, motor_hw[motor].in2_pin, GPIO_PIN_RESET);

		motor_state[motor].last_count = __HAL_TIM_GET_COUNTER(motor_hw[motor].encoder_timer);

		if (HAL_TIM_PWM_Start(motor_hw[motor].pwm_timer, motor_hw[motor].pwm_channel) != HAL_OK)
		{
			Error_Handler();
		}

		if (HAL_TIM_Encoder_Start(motor_hw[motor].encoder_timer, TIM_CHANNEL_ALL) != HAL_OK)
		{
			Error_Handler();
		}

		__HAL_TIM_SET_COMPARE(motor_hw[motor].pwm_timer, motor_hw[motor].pwm_channel, 0);
	}
}

void Motor_StopAll(void)
{
	for (MotorId_t motor = MOTOR_RIGHT_REAR; motor < MOTOR_COUNT; motor++)
	{
		__HAL_TIM_SET_COMPARE(motor_hw[motor].pwm_timer, motor_hw[motor].pwm_channel, 0);
		HAL_GPIO_WritePin(motor_hw[motor].in1_port, motor_hw[motor].in1_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor_hw[motor].in2_port, motor_hw[motor].in2_pin, GPIO_PIN_RESET);
	}
}

void Motor_SetInversion(MotorId_t motor, uint8_t inverted)
{
	motor = motor_valid_id(motor);
	motor_state[motor].inverted = inverted ? 1U : 0U;
}

void Motor_SetPIDGain(MotorId_t motor, float kp, float ki, float kd)
{
	motor = motor_valid_id(motor);
	motor_state[motor].pid.kp = kp;
	motor_state[motor].pid.ki = ki;
	motor_state[motor].pid.kd = kd;
}

void Motor_SetTarget(MotorId_t motor, float target)
{
	motor = motor_valid_id(motor);
	motor_state[motor].pid.target = target;
}

void Motor_SetTargetPercent(MotorId_t motor, float percent)
{
	motor = motor_valid_id(motor);

	if (percent < -100.0f)
	{
		percent = -100.0f;
	}
	else if (percent > 100.0f)
	{
		percent = 100.0f;
	}

	motor_state[motor].pid.target = (percent * MOTOR_MAX_SPEED_COUNTS_PER_SEC) / 100.0f;
}

void Motor_SetOutputLimit(MotorId_t motor, float min_output, float max_output)
{
	motor = motor_valid_id(motor);

	if (min_output > max_output)
	{
		float temp = min_output;
		min_output = max_output;
		max_output = temp;
	}

	motor_state[motor].pid.output_min = min_output;
	motor_state[motor].pid.output_max = max_output;
}

void Motor_ResetPID(MotorId_t motor)
{
	motor = motor_valid_id(motor);
	motor_state[motor].pid.error = 0.0f;
	motor_state[motor].pid.integral = 0.0f;
	motor_state[motor].pid.last_error = 0.0f;
	motor_state[motor].pid.output = 0.0f;
}

void Motor_SetRawPWM(MotorId_t motor, int32_t pwm)
{
	motor = motor_valid_id(motor);

	pwm = motor_clamp_int32(pwm, -(int32_t)MOTOR_PWM_PERIOD, (int32_t)MOTOR_PWM_PERIOD);

	if (pwm == 0)
	{
		__HAL_TIM_SET_COMPARE(motor_hw[motor].pwm_timer, motor_hw[motor].pwm_channel, 0);
		HAL_GPIO_WritePin(motor_hw[motor].in1_port, motor_hw[motor].in1_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor_hw[motor].in2_port, motor_hw[motor].in2_pin, GPIO_PIN_RESET);
		return;
	}

	if (pwm > 0)
	{
		motor_apply_direction(motor, 1U);
	}
	else
	{
		motor_apply_direction(motor, 0U);
		pwm = -pwm;
	}

	__HAL_TIM_SET_COMPARE(motor_hw[motor].pwm_timer, motor_hw[motor].pwm_channel, (uint32_t)pwm);
}

int32_t Motor_GetEncoderDelta(MotorId_t motor)
{
	motor = motor_valid_id(motor);
	return motor_read_delta(motor);
}

float Motor_GetFeedback(MotorId_t motor)
{
	motor = motor_valid_id(motor);
	return motor_state[motor].pid.feedback;
}

const MotorPid_t *Motor_GetPID(MotorId_t motor)
{
	motor = motor_valid_id(motor);
	return &motor_state[motor].pid;
}

void Motor_UpdateControl(float dt_s)
{
	if (dt_s <= 0.0f)
	{
		return;
	}

	for (MotorId_t motor = MOTOR_RIGHT_REAR; motor < MOTOR_COUNT; motor++)
	{
		MotorPid_t *pid = &motor_state[motor].pid;
		float delta = (float)motor_read_delta(motor);

		pid->feedback = delta / dt_s;
		pid->error = pid->target - pid->feedback;
		pid->integral += pid->error * dt_s;

		float derivative = (pid->error - pid->last_error) / dt_s;
		float output = (pid->kp * pid->error) + (pid->ki * pid->integral) + (pid->kd * derivative);

		output = motor_clamp_float(output, pid->output_min, pid->output_max);
		pid->output = output;
		pid->last_error = pid->error;

		Motor_SetRawPWM(motor, (int32_t)output);
	}
}

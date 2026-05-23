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
	uint8_t encoder_inverted;
	uint32_t last_count;
	int32_t position; /* cumulative encoder counts */
	int32_t last_delta;
	float feedback_raw;
	float feedback_filtered;
	uint8_t feedback_lpf_initialized;
	/* position PID */
	int32_t pos_target;
	uint8_t pos_active;
	float pos_kp;
	float pos_ki;
	float pos_kd;
	float pos_integral;
	float pos_last_error;
	float pos_output_limit;
	float target_bias_counts_per_sec;
} MotorState_t;

/* config.h provides MOTOR_PWM_PERIOD and MOTOR_MAX_SPEED_COUNTS_PER_SEC */

static const MotorHardware_t motor_hw[MOTOR_COUNT] =
{
	{&htim1, TIM_CHANNEL_1, &htim5, GPIOE, GPIO_PIN_7,  GPIOE, GPIO_PIN_8},
	{&htim1, TIM_CHANNEL_2, &htim2, GPIOA, GPIO_PIN_9,  GPIOA, GPIO_PIN_8},
	{&htim1, TIM_CHANNEL_3, &htim3, GPIOE, GPIO_PIN_12, GPIOE, GPIO_PIN_10},
	{&htim1, TIM_CHANNEL_4, &htim4, GPIOC, GPIO_PIN_6,  GPIOC, GPIO_PIN_8},
};

static MotorState_t motor_state[MOTOR_COUNT];

static float motor_low_pass_update(float previous, float input, float alpha)
{
	if (alpha <= 0.0f)
	{
		return previous;
	}

	if (alpha >= 1.0f)
	{
		return input;
	}

	return previous + alpha * (input - previous);
}

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

	if (motor_state[motor].encoder_inverted)
	{
		signed_diff = -signed_diff;
	}

	motor_state[motor].last_count = current;
	motor_state[motor].last_delta = signed_diff;
	/* position accumulation will be handled by caller (to allow unified sampling) */
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
	/* Per-wheel velocity PID defaults from named macros (RR, RF, LF, LR) */
	const float kp_arr[MOTOR_COUNT] = { VELOCITY_PID_KP_RR, VELOCITY_PID_KP_RF, VELOCITY_PID_KP_LF, VELOCITY_PID_KP_LR };
	const float ki_arr[MOTOR_COUNT] = { VELOCITY_PID_KI_RR, VELOCITY_PID_KI_RF, VELOCITY_PID_KI_LF, VELOCITY_PID_KI_LR };
	const float kd_arr[MOTOR_COUNT] = { VELOCITY_PID_KD_RR, VELOCITY_PID_KD_RF, VELOCITY_PID_KD_LF, VELOCITY_PID_KD_LR };

	for (MotorId_t motor = MOTOR_RIGHT_REAR; motor < MOTOR_COUNT; motor++)
	{
		motor_state[motor].inverted = 0;
		motor_state[motor].encoder_inverted = 0;
		motor_state[motor].last_count = 0;
		/* Initialize velocity PID defaults from config (per-wheel) */
		motor_state[motor].pid.kp = kp_arr[motor];
		motor_state[motor].pid.ki = ki_arr[motor];
		motor_state[motor].pid.kd = kd_arr[motor];
		motor_state[motor].pid.target = 0.0f;
		motor_state[motor].pid.feedback = 0.0f;
		motor_state[motor].feedback_raw = 0.0f;
		motor_state[motor].feedback_filtered = 0.0f;
		motor_state[motor].feedback_lpf_initialized = 0U;
		motor_state[motor].pid.error = 0.0f;
		motor_state[motor].pid.integral = 0.0f;
		motor_state[motor].pid.last_error = 0.0f;
		motor_state[motor].pid.output = 0.0f;
		motor_state[motor].pid.output_min = -MOTOR_PWM_PERIOD;
		motor_state[motor].pid.output_max = MOTOR_PWM_PERIOD;

		HAL_GPIO_WritePin(motor_hw[motor].in1_port, motor_hw[motor].in1_pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(motor_hw[motor].in2_port, motor_hw[motor].in2_pin, GPIO_PIN_RESET);

		motor_state[motor].last_count = __HAL_TIM_GET_COUNTER(motor_hw[motor].encoder_timer);
		motor_state[motor].position = 0;
		motor_state[motor].last_delta = 0;

		motor_state[motor].pos_target = 0;
		motor_state[motor].pos_active = 0;
		/* Position PID defaults */
		motor_state[motor].pos_kp = POSITION_PID_KP;
		motor_state[motor].pos_ki = POSITION_PID_KI;
		motor_state[motor].pos_kd = POSITION_PID_KD;
		motor_state[motor].pos_integral = 0.0f;
		motor_state[motor].pos_last_error = 0.0f;
		motor_state[motor].pos_output_limit = POSITION_OUTPUT_LIMIT;
		motor_state[motor].target_bias_counts_per_sec = 0.0f;

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

void Motor_SetEncoderInversion(MotorId_t motor, uint8_t inverted)
{
	motor = motor_valid_id(motor);
	motor_state[motor].encoder_inverted = inverted ? 1U : 0U;
}

int32_t Motor_GetEncoderDelta(MotorId_t motor)
{
	motor = motor_valid_id(motor);
	return motor_read_delta(motor);
}

float Motor_GetFeedback(MotorId_t motor)
{
	motor = motor_valid_id(motor);
	return motor_state[motor].feedback_raw;

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

	/* First: sample encoder deltas for all motors and update positions/feedback */
	for (MotorId_t motor = MOTOR_RIGHT_REAR; motor < MOTOR_COUNT; motor++)
	{
		int32_t d = motor_read_delta(motor);
		motor_state[motor].position += d;
		motor_state[motor].feedback_raw = (float)d / dt_s;
		if (!motor_state[motor].feedback_lpf_initialized)
		{
			motor_state[motor].feedback_filtered = motor_state[motor].feedback_raw;
			motor_state[motor].feedback_lpf_initialized = 1U;
		}
		else
		{
			motor_state[motor].feedback_filtered = motor_low_pass_update(motor_state[motor].feedback_filtered,
																	 motor_state[motor].feedback_raw,
																	 MOTOR_FEEDBACK_LPF_ALPHA);
			}
		motor_state[motor].pid.feedback = motor_state[motor].feedback_raw;
	}

	/* Position (outer) -> Velocity (inner) cascade and velocity PID */
	for (MotorId_t motor = MOTOR_RIGHT_REAR; motor < MOTOR_COUNT; motor++)
	{
		MotorPid_t *pid = &motor_state[motor].pid;

		/* If position control active, compute desired velocity setpoint from position PID */
		if (motor_state[motor].pos_active)
		{
			float pos_err = (float)(motor_state[motor].pos_target - motor_state[motor].position);
			float pos_deriv = (pos_err - motor_state[motor].pos_last_error) / dt_s;
			float desired = (motor_state[motor].pos_kp * pos_err) + (motor_state[motor].pos_ki * motor_state[motor].pos_integral) + (motor_state[motor].pos_kd * pos_deriv);

			/* anti-windup and integral update: only integrate if not saturated */
			float unclamped = desired;
			if (unclamped > motor_state[motor].pos_output_limit) unclamped = motor_state[motor].pos_output_limit;
			if (unclamped < -motor_state[motor].pos_output_limit) unclamped = -motor_state[motor].pos_output_limit;

			/* update integral using pos_err */
			motor_state[motor].pos_integral += pos_err * dt_s;
			motor_state[motor].pos_last_error = pos_err;

			/* set inner-loop velocity target (counts/sec) */
			/* clamp to max motor speed */
			if (desired > MOTOR_MAX_SPEED_COUNTS_PER_SEC) desired = MOTOR_MAX_SPEED_COUNTS_PER_SEC;
			if (desired < -MOTOR_MAX_SPEED_COUNTS_PER_SEC) desired = -MOTOR_MAX_SPEED_COUNTS_PER_SEC;
			pid->target = desired;
		}

		/* Compute effective target = base target + bias (no accumulation across cycles) */
		float effective_target = pid->target;
		if (motor_state[motor].target_bias_counts_per_sec != 0.0f)
		{
			effective_target += motor_state[motor].target_bias_counts_per_sec;
			if (effective_target > MOTOR_MAX_SPEED_COUNTS_PER_SEC)
			{
				effective_target = MOTOR_MAX_SPEED_COUNTS_PER_SEC;
			}
			else if (effective_target < -MOTOR_MAX_SPEED_COUNTS_PER_SEC)
			{
				effective_target = -MOTOR_MAX_SPEED_COUNTS_PER_SEC;
			}
		}

		/* Safety: if velocity target is zero and no position active, force outputs 0 */
		if ((pid->target == 0.0f) && (motor_state[motor].pos_active == 0) && (motor_state[motor].target_bias_counts_per_sec == 0.0f))
		{
			pid->error = 0.0f;
			pid->integral = 0.0f;
			pid->last_error = 0.0f;
			pid->output = 0.0f;
			Motor_SetRawPWM(motor, 0);
			continue;
		}

		/* Velocity PID */
		float error = effective_target - motor_state[motor].feedback_filtered;
		pid->error = error;
		pid->integral += error * dt_s;
		float derivative = (error - pid->last_error) / dt_s;
		float output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
		output = motor_clamp_float(output, pid->output_min, pid->output_max);
		pid->output = output;
		pid->last_error = error;

		Motor_SetRawPWM(motor, (int32_t)output);
	}
}

/* Position control API implementations */
void Motor_SetPositionPIDGain(MotorId_t motor, float kp, float ki, float kd)
{
	motor = motor_valid_id(motor);
	motor_state[motor].pos_kp = kp;
	motor_state[motor].pos_ki = ki;
	motor_state[motor].pos_kd = kd;
}

void Motor_SetPositionTarget(MotorId_t motor, int32_t counts_relative)
{
	motor = motor_valid_id(motor);
	motor_state[motor].pos_target = motor_state[motor].position + counts_relative;
	motor_state[motor].pos_integral = 0.0f;
	motor_state[motor].pos_last_error = 0.0f;
	motor_state[motor].pos_active = 1;
}

void Motor_ClearPositionTarget(MotorId_t motor)
{
	motor = motor_valid_id(motor);
	motor_state[motor].pos_active = 0;
	motor_state[motor].pos_integral = 0.0f;
	motor_state[motor].pos_last_error = 0.0f;
	motor_state[motor].target_bias_counts_per_sec = 0.0f;
	/* clear inner-loop target and reset velocity PID integrator to avoid immediate reactivation */
	motor_state[motor].pid.target = 0.0f;
	motor_state[motor].pid.integral = 0.0f;
	motor_state[motor].pid.error = 0.0f;
	motor_state[motor].pid.last_error = 0.0f;
	motor_state[motor].pid.output = 0.0f;
	Motor_SetRawPWM(motor, 0);
}

int32_t Motor_GetPosition(MotorId_t motor)
{
	motor = motor_valid_id(motor);
	return motor_state[motor].position;
}

uint8_t Motor_PositionReached(MotorId_t motor, int32_t tolerance)
{
	motor = motor_valid_id(motor);
	if (!motor_state[motor].pos_active) return 1;
	int32_t err = motor_state[motor].pos_target - motor_state[motor].position;
	if (err < 0) err = -err;
	return (err <= tolerance) ? 1 : 0;
}

uint8_t Motor_HasActivePositionTarget(void)
{
	for (MotorId_t motor = MOTOR_RIGHT_REAR; motor < MOTOR_COUNT; motor++)
	{
		if (motor_state[motor].pos_active)
		{
			return 1U;
		}
	}

	return 0U;
}

void Motor_SetTargetBiasPercent(MotorId_t motor, float percent_bias)
{
	motor = motor_valid_id(motor);
	if (percent_bias < -100.0f)
	{
		percent_bias = -100.0f;
	}
	else if (percent_bias > 100.0f)
	{
		percent_bias = 100.0f;
	}

	motor_state[motor].target_bias_counts_per_sec = (percent_bias * MOTOR_MAX_SPEED_COUNTS_PER_SEC) / 100.0f;
}

void Motor_ClearTargetBias(MotorId_t motor)
{
	motor = motor_valid_id(motor);
	motor_state[motor].target_bias_counts_per_sec = 0.0f;
}

void Motor_PositionUpdate(float dt_s)
{
	/* Alias for Motor_UpdateControl for compatibility (outer loop handled there) */
	Motor_UpdateControl(dt_s);
}

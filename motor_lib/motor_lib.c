#include "motor_lib.h"
#include "Pwm_lib.h"

void motor_control(Motor_TypeDef *motor, MotorState state, uint8_t speed )
{
	switch(state)// state
	{
		case MOTOR_STOP:
			HAL_GPIO_WritePin(motor->io_port, motor->io_pin, GPIO_PIN_RESET);
			pwm_set_duty(motor->tim,motor->tim_channel, 0);
			break;
		case MOTOR_CW:
			HAL_GPIO_WritePin(motor->io_port, motor->io_pin, GPIO_PIN_RESET);
			pwm_set_duty(motor->tim,motor->tim_channel, speed);
			break;
		case MOTOR_CCW:
			HAL_GPIO_WritePin(motor->io_port, motor->io_pin, GPIO_PIN_SET);
			pwm_set_duty(motor->tim,motor->tim_channel, 100 - speed);
			break;
	}
}

void motor_init(Motor_TypeDef *motor,GPIO_TypeDef *io_port,
		uint16_t io_pin,TIM_HandleTypeDef *tim,uint32_t tim_channel)
{
	motor->io_port = io_port;
	motor->io_pin  = io_pin;
	motor->tim = tim;
	motor->tim_channel = tim_channel;
	HAL_TIM_PWM_Start(motor->tim, motor->tim_channel);
}



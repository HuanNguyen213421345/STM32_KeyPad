#ifndef MOTOR_LIB_H_
#define MOTOR_LIB_H_

#include "stm32f1xx_hal.h"
#include "stdint.h"

typedef enum
{
	MOTOR_STOP,
	MOTOR_CW,// cung chieu kim dong ho.
	MOTOR_CCW // nguoc chieu kim dong ho.
}MotorState;

typedef struct
{
	GPIO_TypeDef *io_port;
	uint16_t io_pin;
	TIM_HandleTypeDef *tim;
	uint32_t tim_channel;
	MotorState motor_state;
	uint8_t speed;
}Motor_TypeDef;

void motor_control(Motor_TypeDef *motor, MotorState state, uint8_t speed);
void motor_init(Motor_TypeDef *motor,GPIO_TypeDef *io_port,uint16_t io_pin,
		TIM_HandleTypeDef *tim,uint32_t tim_channel );

#endif /* MOTOR_LIB_H_ */

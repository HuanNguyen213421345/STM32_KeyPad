/*
 * Pwm_lib.h
 *
 *  Created on: Nov 13, 2024
 *      Author: ASUS
 */

#ifndef PWM_LIB_H_
#define PWM_LIB_H_

#include "stdint.h"
#include "stm32f1xx_hal.h"


void pwm_set_duty(TIM_HandleTypeDef *tim, uint16_t channel, uint8_t duty);


#endif /* PWM_LIB_H_ */

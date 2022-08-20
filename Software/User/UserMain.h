/*
 * UserMain.h
 *
 *  Created on: Jan 11, 2021
 *      Author: huang
 */


#ifndef _USERMAIN_H_
#define _USERMAIN_H_

#include "stm32f4xx_hal.h"

#include <stdint.h>
#include <stdio.h>


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart2;

extern I2C_HandleTypeDef hi2c1;

#define pwm_left_channel TIM_CHANNEL_2
#define pwm_right_channel TIM_CHANNEL_3

#define ain1_Pin GPIO_PIN_12
#define ain2_Pin GPIO_PIN_13
#define bin1_Pin GPIO_PIN_14
#define bin2_Pin GPIO_PIN_15

void UserSysInit();

void UserLoop();

void UserConfigInit();

#endif /* _USERMAIN_H_ */

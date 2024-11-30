/*
 * delay_timer.h
 *
 *  Created on: Nov 3, 2024
 *      Author: buikh
 */


#ifndef __DELAY_TIMER_H
#define __DELAY_TIMER_H
#include "stm32f1xx_hal.h"
void DELAY_TIM_Init(TIM_HandleTypeDef *htim);
void DELAY_TIM_Us(TIM_HandleTypeDef *htim, uint16_t time);
void DELAY_TIM_Ms(TIM_HandleTypeDef *htim, uint16_t Time);
#endif

#ifndef ACTUATOR_DRIVER_H
#define ACTUATOR_DRIVER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void Actuator_Init(TIM_HandleTypeDef *htim_clamp,
                   TIM_HandleTypeDef *htim_lift);

void Actuator_StopAll(void);

/* KẸP - BTS GIỮA */
void Clamp_Stop(void);
void Clamp_Open(uint16_t pwm);
void Clamp_Close(uint16_t pwm);

/* NÂNG/HẠ - BTS TRÁI + PHẢI */
void Lift_Stop(void);
void Lift_Up(uint16_t pwm);
void Lift_Down(uint16_t pwm);

#endif

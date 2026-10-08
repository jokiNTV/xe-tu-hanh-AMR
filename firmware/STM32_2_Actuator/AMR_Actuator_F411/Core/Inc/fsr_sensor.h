#ifndef FSR_SENSOR_H
#define FSR_SENSOR_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void FSR_Init(ADC_HandleTypeDef *hadc);

void FSR_Update(void);

uint32_t FSR_GetRaw(void);
uint32_t FSR_GetFiltered(void);

uint8_t FSR_IsContact(void);
uint8_t FSR_IsClampOK(void);
uint8_t FSR_IsOverForce(void);

#endif

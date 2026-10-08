#ifndef HCSR04_H
#define HCSR04_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Khởi tạo HC-SR04 */
void HCSR04_Init(TIM_HandleTypeDef *htim_us);

/* Đọc và cập nhật khoảng cách */
void HCSR04_Update(void);

/* Khoảng cách cm */
float HCSR04_GetDistanceCm(void);

/* 1 = dữ liệu hợp lệ, 0 = lỗi */
uint8_t HCSR04_IsValid(void);

/*
0 = OK
1 = ECHO không lên HIGH
2 = ECHO lên HIGH nhưng không xuống LOW
3 = khoảng cách ngoài vùng 2-400 cm
*/
uint8_t HCSR04_GetError(void);

/* Độ rộng xung ECHO thực tế, đơn vị microsecond */
uint32_t HCSR04_GetPulseUs(void);

#endif

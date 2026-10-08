#include "hcsr04.h"


/* =========================================================
 * HARDWARE
 * TRIG = PB10
 * ECHO = PB12
 * TIM2 = 1 MHz -> 1 count = 1 us
 * ========================================================= */

static TIM_HandleTypeDef *s_htim = NULL;


/* =========================================================
 * DATA
 * ========================================================= */

static float s_distance_cm = -1.0f;

static uint8_t s_valid = 0;

static uint8_t s_error = 0;

static uint32_t s_pulse_us = 0;


/* =========================================================
 * MICROSECOND DELAY
 * ========================================================= */

static void Delay_us(uint32_t us)
{
    if (s_htim == NULL)
        return;

    __HAL_TIM_SET_COUNTER(s_htim, 0);

    while (__HAL_TIM_GET_COUNTER(s_htim) < us)
    {
        /* wait */
    }
}


/* =========================================================
 * INIT
 * ========================================================= */

void HCSR04_Init(TIM_HandleTypeDef *htim_us)
{
    s_htim = htim_us;

    s_distance_cm = -1.0f;
    s_valid = 0;
    s_error = 0;
    s_pulse_us = 0;

    HAL_TIM_Base_Start(s_htim);

    /* TRIG luôn LOW lúc khởi động */
    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_10,
        GPIO_PIN_RESET
    );
}


/* =========================================================
 * UPDATE
 * ========================================================= */

void HCSR04_Update(void)
{
    uint32_t pulse_us;
    float distance;


    if (s_htim == NULL)
    {
        s_valid = 0;
        s_error = 1;
        s_distance_cm = -1.0f;
        s_pulse_us = 0;
        return;
    }


    /* Reset trạng thái lỗi */
    s_error = 0;


    /* =====================================================
     * PHÁT TRIG
     * ===================================================== */

    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_10,
        GPIO_PIN_RESET
    );

    Delay_us(3);


    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_10,
        GPIO_PIN_SET
    );

    Delay_us(10);


    HAL_GPIO_WritePin(
        GPIOB,
        GPIO_PIN_10,
        GPIO_PIN_RESET
    );


    /* =====================================================
     * CHỜ ECHO LÊN HIGH
     * ===================================================== */

    __HAL_TIM_SET_COUNTER(s_htim, 0);

    while (
        HAL_GPIO_ReadPin(
            GPIOB,
            GPIO_PIN_12
        ) == GPIO_PIN_RESET
    )
    {
        if (__HAL_TIM_GET_COUNTER(s_htim) > 30000U)
        {
            s_valid = 0;
            s_error = 1;

            s_distance_cm = -1.0f;
            s_pulse_us = 0;

            return;
        }
    }


    /* =====================================================
     * ĐO ĐỘ RỘNG XUNG ECHO HIGH
     * ===================================================== */

    __HAL_TIM_SET_COUNTER(s_htim, 0);

    while (
        HAL_GPIO_ReadPin(
            GPIOB,
            GPIO_PIN_12
        ) == GPIO_PIN_SET
    )
    {
        if (__HAL_TIM_GET_COUNTER(s_htim) > 30000U)
        {
            s_valid = 0;
            s_error = 2;

            s_distance_cm = -1.0f;
            s_pulse_us = 0;

            return;
        }
    }


    pulse_us =
        __HAL_TIM_GET_COUNTER(s_htim);


    /* Lưu xung thật để Debug */
    s_pulse_us = pulse_us;


    /* =====================================================
     * TÍNH KHOẢNG CÁCH
     *
     * tốc độ âm thanh ≈ 0.0343 cm/us
     * chia 2 vì sóng đi và quay lại
     * ===================================================== */

    distance =
        ((float)pulse_us * 0.0343f)
        / 2.0f;


    /* =====================================================
     * VALIDATE
     * ===================================================== */

    if (
        (distance >= 2.0f)
        &&
        (distance <= 400.0f)
    )
    {
        /*
         Lọc nhẹ.

         Nếu chưa có giá trị hợp lệ trước đó,
         lấy luôn giá trị mới.
        */

        if (s_distance_cm < 0.0f)
        {
            s_distance_cm = distance;
        }
        else
        {
            s_distance_cm =
                (0.70f * s_distance_cm)
                +
                (0.30f * distance);
        }

        s_valid = 1;
        s_error = 0;
    }
    else
    {
        s_valid = 0;
        s_error = 3;

        s_distance_cm = -1.0f;
    }
}


/* =========================================================
 * GETTERS
 * ========================================================= */

float HCSR04_GetDistanceCm(void)
{
    return s_distance_cm;
}


uint8_t HCSR04_IsValid(void)
{
    return s_valid;
}


uint8_t HCSR04_GetError(void)
{
    return s_error;
}


uint32_t HCSR04_GetPulseUs(void)
{
    return s_pulse_us;
}

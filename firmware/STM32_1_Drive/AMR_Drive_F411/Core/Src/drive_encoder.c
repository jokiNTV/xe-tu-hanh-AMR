#include "drive_encoder.h"

#include <stddef.h>

#include "stm32f4xx_hal.h"
#include "tim.h"

#if ((DRIVE_ENCODER_LEFT_SIGN != 1) && (DRIVE_ENCODER_LEFT_SIGN != -1))
#error "DRIVE_ENCODER_LEFT_SIGN must be 1 or -1"
#endif

#if ((DRIVE_ENCODER_RIGHT_SIGN != 1) && (DRIVE_ENCODER_RIGHT_SIGN != -1))
#error "DRIVE_ENCODER_RIGHT_SIGN must be 1 or -1"
#endif

typedef struct
{
    uint32_t left_previous;
    uint16_t right_previous;
    int64_t left_total;
    int64_t right_total;
    bool initialized;
} DriveEncoderState_t;

static DriveEncoderState_t encoder_state;

static bool timer_periods_are_valid(void)
{
    return ((__HAL_TIM_GET_AUTORELOAD(&htim2) == 0xFFFFFFFFUL) &&
            (__HAL_TIM_GET_AUTORELOAD(&htim3) == 0xFFFFUL));
}

bool DriveEncoder_Init(void)
{
    encoder_state.initialized = false;

    if (!timer_periods_are_valid())
    {
        return false;
    }

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);

    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
    {
        (void)HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
        return false;
    }

    encoder_state.left_previous = 0U;
    encoder_state.right_previous = 0U;
    encoder_state.left_total = 0;
    encoder_state.right_total = 0;
    encoder_state.initialized = true;

    return true;
}

void DriveEncoder_Stop(void)
{
    if (!encoder_state.initialized)
    {
        return;
    }

    (void)HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
    encoder_state.initialized = false;
}

void DriveEncoder_Reset(void)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    __HAL_TIM_SET_COUNTER(&htim3, 0U);

    encoder_state.left_previous = 0U;
    encoder_state.right_previous = 0U;
    encoder_state.left_total = 0;
    encoder_state.right_total = 0;
}

bool DriveEncoder_Sample(DriveEncoderSample_t *sample)
{
    uint32_t left_now;
    uint16_t right_now;
    int32_t left_delta;
    int32_t right_delta;

    if ((sample == NULL) || !encoder_state.initialized)
    {
        return false;
    }

    left_now = (uint32_t)__HAL_TIM_GET_COUNTER(&htim2);
    right_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);

    /*
     * Ep kieu sau phep tru de xu ly tran bo dem:
     * - TIM2: modulo 2^32.
     * - TIM3: modulo 2^16.
     * Dieu kien: so xung giua hai lan lay mau khong vuot nua mien dem.
     */
    left_delta = (int32_t)(left_now - encoder_state.left_previous);
    right_delta = (int32_t)(int16_t)
                  (uint16_t)(right_now - encoder_state.right_previous);

    left_delta *= DRIVE_ENCODER_LEFT_SIGN;
    right_delta *= DRIVE_ENCODER_RIGHT_SIGN;

    encoder_state.left_previous = left_now;
    encoder_state.right_previous = right_now;
    encoder_state.left_total += left_delta;
    encoder_state.right_total += right_delta;

    sample->left_total_count = encoder_state.left_total;
    sample->right_total_count = encoder_state.right_total;
    sample->left_delta_count = left_delta;
    sample->right_delta_count = right_delta;
    sample->timestamp_ms = HAL_GetTick();

    return true;
}

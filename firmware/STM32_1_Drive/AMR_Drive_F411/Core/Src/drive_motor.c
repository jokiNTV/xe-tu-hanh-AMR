#include "drive_motor.h"
#include "tim.h"

static int16_t ClampPwm(int16_t pwm)
{
    if (pwm > (int16_t)DRIVE_PWM_MAX)
    {
        return (int16_t)DRIVE_PWM_MAX;
    }

    if (pwm < -(int16_t)DRIVE_PWM_MAX)
    {
        return -(int16_t)DRIVE_PWM_MAX;
    }

    return pwm;
}

static void SetLeftMotor(int16_t pwm)
{
    pwm = ClampPwm(pwm);

    /* Tat ca hai chieu truoc khi chon chieu moi. */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0U);

    /*
     * LEFT direction fixed 2026-08-28:
     * Physical LEFT motor wiring requires the TIM4 channels inverted
     * relative to the previous software mapping.
     *
     * +PWM -> TIM4_CH2
     * -PWM -> TIM4_CH1
     */
    if (pwm > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim4,
                              TIM_CHANNEL_2,
                              (uint16_t)pwm);
    }
    else if (pwm < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim4,
                              TIM_CHANNEL_1,
                              (uint16_t)(-pwm));
    }
}

static void SetRightMotor(int16_t pwm)
{
    pwm = ClampPwm(pwm);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);

    /*
     * Motor phai lap doi xung voi motor trai:
     * +PWM -> PA9/TIM1_CH2  -> xe tien.
     * -PWM -> PA8/TIM1_CH1  -> xe lui.
     */
    if (pwm > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1,
                              TIM_CHANNEL_2,
                              (uint16_t)pwm);
    }
    else if (pwm < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim1,
                              TIM_CHANNEL_1,
                              (uint16_t)(-pwm));
    }
}

bool DriveMotor_Init(void)
{
    DriveMotor_Stop();

    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    /* PA9 la TIM1_CH2 thuong, khong dung PB0/TIM1_CH2N. */
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK)
    {
        return false;
    }

    DriveMotor_Stop();
    return true;
}

void DriveMotor_Set(int16_t left_pwm, int16_t right_pwm)
{
    SetLeftMotor(left_pwm);
    SetRightMotor(right_pwm);
}

void DriveMotor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
}

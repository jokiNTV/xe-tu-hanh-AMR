#include "actuator_driver.h"
#include "mechanism_config.h"

static TIM_HandleTypeDef *s_tim_clamp = NULL;
static TIM_HandleTypeDef *s_tim_lift = NULL;


static uint16_t PWM_Limit(uint16_t pwm)
{
    if (pwm > ACT_PWM_MAX)
        pwm = ACT_PWM_MAX;

    return pwm;
}


void Actuator_Init(TIM_HandleTypeDef *htim_clamp,
                   TIM_HandleTypeDef *htim_lift)
{
    s_tim_clamp = htim_clamp;
    s_tim_lift = htim_lift;

    /* TIM1 - BTS GIỮA */
    HAL_TIM_PWM_Start(s_tim_clamp, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(s_tim_clamp, TIM_CHANNEL_2);

    /* TIM4 - BTS TRÁI + PHẢI */
    HAL_TIM_PWM_Start(s_tim_lift, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(s_tim_lift, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(s_tim_lift, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(s_tim_lift, TIM_CHANNEL_4);

    Actuator_StopAll();
}


/* =========================================================
   BTS GIỮA - KẸP
   PA8 = TIM1_CH1
   PA9 = TIM1_CH2
   ========================================================= */

void Clamp_Stop(void)
{
    __HAL_TIM_SET_COMPARE(s_tim_clamp, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(s_tim_clamp, TIM_CHANNEL_2, 0);
}


void Clamp_Open(uint16_t pwm)
{
    pwm = PWM_Limit(pwm);

#if CLAMP_OPEN_CH1
    __HAL_TIM_SET_COMPARE(s_tim_clamp, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(s_tim_clamp, TIM_CHANNEL_2, 0);
#else
    __HAL_TIM_SET_COMPARE(s_tim_clamp, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(s_tim_clamp, TIM_CHANNEL_2, pwm);
#endif
}


void Clamp_Close(uint16_t pwm)
{
    pwm = PWM_Limit(pwm);

#if CLAMP_OPEN_CH1
    __HAL_TIM_SET_COMPARE(s_tim_clamp, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(s_tim_clamp, TIM_CHANNEL_2, pwm);
#else
    __HAL_TIM_SET_COMPARE(s_tim_clamp, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(s_tim_clamp, TIM_CHANNEL_2, 0);
#endif
}


/* =========================================================
   2 XI LANH NÂNG
   LEFT:
   PB6 = TIM4_CH1
   PB7 = TIM4_CH2

   RIGHT:
   PB8 = TIM4_CH3
   PB9 = TIM4_CH4
   ========================================================= */

void Lift_Stop(void)
{
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_2, 0);

    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_4, 0);
}


void Lift_Up(uint16_t pwm)
{
    pwm = PWM_Limit(pwm);

    /* LEFT */
#if LEFT_UP_CH1
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_2, 0);
#else
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_2, pwm);
#endif

    /* RIGHT */
#if RIGHT_UP_CH3
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_3, pwm);
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_4, 0);
#else
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_4, pwm);
#endif
}


void Lift_Down(uint16_t pwm)
{
    pwm = PWM_Limit(pwm);

    /* LEFT */
#if LEFT_UP_CH1
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_2, pwm);
#else
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_1, pwm);
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_2, 0);
#endif

    /* RIGHT */
#if RIGHT_UP_CH3
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_4, pwm);
#else
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_3, pwm);
    __HAL_TIM_SET_COMPARE(s_tim_lift, TIM_CHANNEL_4, 0);
#endif
}


void Actuator_StopAll(void)
{
    Clamp_Stop();
    Lift_Stop();
}

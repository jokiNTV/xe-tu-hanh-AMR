#include "fsr_sensor.h"
#include "mechanism_config.h"

static ADC_HandleTypeDef *s_hadc = NULL;

static uint32_t s_raw = 0;
static uint32_t s_filtered = 0;

static uint8_t s_initialized = 0;


void FSR_Init(ADC_HandleTypeDef *hadc)
{
    s_hadc = hadc;

    s_raw = 0;
    s_filtered = 0;
    s_initialized = 0;
}


void FSR_Update(void)
{
    if (s_hadc == NULL)
        return;

    uint32_t sum = 0;
    uint32_t valid_samples = 0;

    /* Trung bình 16 mẫu */
    for (uint8_t i = 0; i < 16; i++)
    {
        HAL_ADC_Start(s_hadc);

        if (HAL_ADC_PollForConversion(s_hadc, 5) == HAL_OK)
        {
            sum += HAL_ADC_GetValue(s_hadc);
            valid_samples++;
        }

        HAL_ADC_Stop(s_hadc);
    }

    if (valid_samples == 0)
        return;

    s_raw = sum / valid_samples;

    /* Low-pass filter */
    if (!s_initialized)
    {
        s_filtered = s_raw;
        s_initialized = 1;
    }
    else
    {
        s_filtered =
            ((s_filtered * 7U) + s_raw) / 8U;
    }
}


uint32_t FSR_GetRaw(void)
{
    return s_raw;
}


uint32_t FSR_GetFiltered(void)
{
    return s_filtered;
}


uint8_t FSR_IsContact(void)
{
    return (s_filtered >= FSR_CONTACT_ADC);
}


uint8_t FSR_IsClampOK(void)
{
    return (s_filtered >= FSR_CLAMP_OK_ADC);
}


uint8_t FSR_IsOverForce(void)
{
    return (s_filtered >= FSR_CLAMP_MAX_ADC);
}

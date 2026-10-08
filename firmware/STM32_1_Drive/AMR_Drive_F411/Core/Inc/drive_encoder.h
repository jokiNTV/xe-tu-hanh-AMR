#ifndef DRIVE_ENCODER_H
#define DRIVE_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Doi thanh -1 sau khi kiem tra neu encoder cua banh do dem nguoc
 * khi robot chay ve phia truoc.
 */
#define DRIVE_ENCODER_LEFT_SIGN   (1)
#define DRIVE_ENCODER_RIGHT_SIGN  (1)

typedef struct
{
    int64_t left_total_count;
    int64_t right_total_count;
    int32_t left_delta_count;
    int32_t right_delta_count;
    uint32_t timestamp_ms;
} DriveEncoderSample_t;

/*
 * TIM2: encoder banh trai, ARR = 0xFFFFFFFF.
 * TIM3: encoder banh phai, ARR = 0xFFFF.
 */
bool DriveEncoder_Init(void);
void DriveEncoder_Stop(void);
void DriveEncoder_Reset(void);
bool DriveEncoder_Sample(DriveEncoderSample_t *sample);

#endif /* DRIVE_ENCODER_H */

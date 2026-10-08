#ifndef MECHANISM_CONFIG_H
#define MECHANISM_CONFIG_H


/* ================= PWM ================= */

#define ACT_PWM_MAX              799U

#define CLAMP_OPEN_PWM           790U
#define CLAMP_CLOSE_PWM          790U
#define LIFT_PWM_DEFAULT         750U
#define CLAMP_CLOSE_FULL_MS      16000U

/* ================= DIRECTION ================= */

#define CLAMP_OPEN_CH1             1U

#define LEFT_UP_CH1                1U

#define RIGHT_UP_CH3               1U


/* ================= FSR402 ================= */

#define FSR_CONTACT_ADC    100U
#define FSR_RECLAMP_ADC    300U
#define FSR_CLAMP_OK_ADC  1750U
#define FSR_CLAMP_MAX_ADC 3000U


/* ================= HC-SR04 ================= */

/* Cao nhất: dừng trước mốc cơ khí 3 cm */
#define LIFT_TOP_STOP_CM          3.5f

/* Thấp nhất: dừng trước mốc cơ khí 8 cm */
#define LIFT_BOTTOM_STOP_CM       9.0f


/* ================= TIME ================= */

#define CLAMP_CONFIRM_MS          200U

#define CLAMP_TIMEOUT_MS         15000U

#define LIFT_UP_TIME_MS         10000U

#define LIFT_DOWN_FULL_TIME_MS  10000U

/* Compatibility aliases for legacy code paths. */
#define LIFT_TIMEOUT_MS          LIFT_UP_TIME_MS
#define LOWER_TIMEOUT_MS         LIFT_DOWN_FULL_TIME_MS

/* Tạm thời, lát nữa hiệu chuẩn thời gian mở kẹp */
#define CLAMP_OPEN_FULL_MS       18500U


#endif /* MECHANISM_CONFIG_H */

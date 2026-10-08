/* USER CODE BEGIN Header */

/**

  ******************************************************************************

  * File Name          : freertos.c

  * Description        : Code for freertos applications

  ******************************************************************************

  * @attention

  *

  * Copyright (c) 2026 STMicroelectronics.

  * All rights reserved.

  *

  * This software is licensed under terms that can be found in the LICENSE file

  * in the root directory of this software component.

  * If no LICENSE file comes with this software, it is provided AS-IS.

  *

  ******************************************************************************

  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "FreeRTOS.h"

#include "task.h"

#include "main.h"

#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include "app_types.h"

#include "amr/amr_messages.h"

#include "app_comm.h"

#include "distance_controller.h"

#include "drive_command_policy.h"

#include "drive_encoder.h"

#include "drive_kinematics.h"

#include "drive_odometry.h"

#include "drive_motor.h"

#include "drive_motion_profile.h"

#include "drive_speed_control.h"

#include "i2c.h"

#include "imu_heading.h"

#include "imu_heading_hold.h"

#include "imu_bno055.h"

#include "turn_controller.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/

typedef StaticTask_t osStaticThreadDef_t;

typedef StaticQueue_t osStaticMessageQDef_t;

typedef StaticEventGroup_t osStaticEventGroupDef_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN PD */

/*
 * V3.2 NORMAL STOP ACTIVE BRAKE
 *
 * A deliberate WHEEL_SPEED 0/0 command keeps the motor-enable authority
 * alive briefly so DriveSpeedControl_Update(0, 0, measured, measured)
 * can execute the V3 active-brake branch before the bridge is released.
 *
 * FAULT / ESTOP / motor-disable paths still stop immediately.
 */
#define DRIVE_NORMAL_STOP_BRAKE_TIMEOUT_MS        (500U)
#define DRIVE_NORMAL_STOP_SETTLE_RAD_S             (0.03f)
#define DRIVE_NORMAL_STOP_SETTLE_HOLD_MS            (40U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN Variables */

volatile int64_t g_encoder_left_count = 0;

volatile int64_t g_encoder_right_count = 0;

volatile int32_t g_encoder_left_delta = 0;

volatile int32_t g_encoder_right_delta = 0;

volatile float g_wheel_left_position_m = 0.0f;

volatile float g_wheel_right_position_m = 0.0f;

volatile float g_wheel_left_speed_m_s = 0.0f;

volatile float g_wheel_right_speed_m_s = 0.0f;

volatile float g_wheel_left_speed_raw_m_s = 0.0f;

volatile float g_wheel_right_speed_raw_m_s = 0.0f;

volatile float g_wheel_left_rpm = 0.0f;

volatile float g_wheel_right_rpm = 0.0f;

volatile float g_wheel_left_speed_rad_s = 0.0f;

volatile float g_wheel_right_speed_rad_s = 0.0f;

volatile float g_drive_target_left_rad_s = 0.0f;

volatile float g_drive_target_right_rad_s = 0.0f;

volatile float g_drive_error_left_rad_s = 0.0f;

volatile float g_drive_error_right_rad_s = 0.0f;

volatile int16_t g_drive_left_pwm = 0;

volatile int16_t g_drive_right_pwm = 0;

volatile uint32_t g_drive_command_age_ms = 0U;

volatile uint32_t g_drive_timeout_count = 0U;

volatile uint8_t g_drive_command_active = 0U;

volatile uint32_t g_drive_command_refresh_count = 0U;

volatile uint32_t g_drive_controller_reset_count = 0U;

volatile uint8_t g_imu_ready = 0U;

volatile uint8_t g_imu_who_am_i = 0U;

volatile uint32_t g_imu_sample_count = 0U;

volatile uint32_t g_imu_error_count = 0U;

volatile uint32_t g_imu_calibration_count = 0U;

volatile uint32_t g_imu_calibration_reset_count = 0U;

volatile float g_imu_accel_x_m_s2 = 0.0f;

volatile float g_imu_accel_y_m_s2 = 0.0f;

volatile float g_imu_accel_z_m_s2 = 0.0f;

volatile float g_imu_temperature_c = 0.0f;

volatile float g_imu_gyro_bias_z_dps = 0.0f;

volatile float g_imu_gyro_z_dps = 0.0f;

volatile float g_imu_gyro_z_rad_s = 0.0f;

volatile float g_imu_yaw_rad = 0.0f;

/* Display-only diagnostics for STM32CubeIDE Live Expressions.

 * These variables are NOT used by any controller or estimator.

 */

volatile float g_imu_yaw_deg = 0.0f;

volatile float g_imu_bno_fused_yaw_deg = 0.0f;

/* User-facing zero command for CubeIDE Live Expressions.

 *

 * Write 1 while the robot is physically aligned with the chosen reference.

 * The IMU task consumes it on a fresh valid sample, makes current yaw = 0 deg,

 * then automatically writes it back to 0.

 *

 * Diagnostics only:

 *   g_imu_zero_count   increments after each successful zero.

 *   g_imu_zero_status  0=idle, 1=success, 2=waiting/not-ready.

 */

volatile uint8_t g_imu_zero_request = 0U;

volatile uint32_t g_imu_zero_count = 0U;

volatile uint8_t g_imu_zero_status = 0U;

/* BNO055 source-filter debug variables.

 * These are diagnostics only; they are not used by control logic.

 */

volatile float g_imu_raw_gyro_z_dps = 0.0f;

volatile float g_imu_filtered_gyro_z_dps = 0.0f;

volatile float g_imu_bno_fused_yaw_rad = 0.0f;

volatile uint32_t g_imu_stationary_count = 0U;

volatile uint32_t g_imu_rejected_orientation_count = 0U;

volatile uint8_t g_heading_hold_active = 0U;

volatile float g_heading_reference_rad = 0.0f;

volatile float g_heading_error_rad = 0.0f;

volatile float g_heading_correction_rad_s = 0.0f;

volatile float g_heading_desired_yaw_rate_rad_s = 0.0f;

volatile float g_heading_yaw_rate_error_rad_s = 0.0f;

volatile float g_heading_rate_ff_correction_rad_s = 0.0f;

volatile float g_heading_rate_p_correction_rad_s = 0.0f;

volatile float g_heading_rate_i_correction_rad_s = 0.0f;

volatile uint8_t g_heading_controller_version =

    (IMU_HEADING_HOLD_V2_ENABLE != 0) ? 2U : 1U;

volatile uint8_t g_heading_correction_saturated = 0U;

volatile uint8_t g_motion_profile_status = DRIVE_MOTION_PROFILE_IDLE;

volatile uint8_t g_motion_profile_fault = 0U;

volatile uint8_t g_turn_status = TURN_CONTROLLER_IDLE;

volatile uint8_t g_turn_active = 0U;

volatile float g_turn_target_angle_rad = 0.0f;

volatile float g_turn_traveled_angle_rad = 0.0f;

volatile float g_turn_error_rad = 0.0f;

volatile float g_turn_yaw_rate_command_rad_s = 0.0f;

volatile uint32_t g_turn_complete_count = 0U;

volatile uint32_t g_turn_timeout_count = 0U;

volatile uint32_t g_turn_imu_fault_count = 0U;

volatile uint8_t g_distance_status = DISTANCE_CONTROLLER_IDLE;

volatile uint8_t g_distance_active = 0U;

volatile float g_distance_target_m = 0.0f;

volatile float g_distance_traveled_m = 0.0f;

volatile float g_distance_remaining_m = 0.0f;

volatile float g_distance_speed_command_m_s = 0.0f;

volatile uint32_t g_distance_complete_count = 0U;

volatile uint32_t g_distance_timeout_count = 0U;

volatile uint32_t g_distance_measurement_fault_count = 0U;

volatile uint8_t g_odom_ready = 0U;

volatile float g_odom_x_m = 0.0f;

volatile float g_odom_y_m = 0.0f;

volatile float g_odom_yaw_rad = 0.0f;

volatile float g_odom_linear_velocity_m_s = 0.0f;

volatile float g_odom_angular_velocity_rad_s = 0.0f;

volatile uint32_t g_odom_sample_count = 0U;

volatile uint32_t g_odom_error_count = 0U;

volatile uint32_t g_odom_reset_count = 0U;

/* USER CODE END Variables */

/* Definitions for SafetyTask */

osThreadId_t SafetyTaskHandle;

uint32_t SafetyTaskBuffer[ 256 ];

osStaticThreadDef_t SafetyTaskControlBlock;

const osThreadAttr_t SafetyTask_attributes = {

  .name = "SafetyTask",

  .cb_mem = &SafetyTaskControlBlock,

  .cb_size = sizeof(SafetyTaskControlBlock),

  .stack_mem = &SafetyTaskBuffer[0],

  .stack_size = sizeof(SafetyTaskBuffer),

  .priority = (osPriority_t) osPriorityRealtime,

};

/* Definitions for MotorCtrlTask */

osThreadId_t MotorCtrlTaskHandle;

uint32_t MotorCtrlTaskBuffer[ 384 ];

osStaticThreadDef_t MotorCtrlTaskControlBlock;

const osThreadAttr_t MotorCtrlTask_attributes = {

  .name = "MotorCtrlTask",

  .cb_mem = &MotorCtrlTaskControlBlock,

  .cb_size = sizeof(MotorCtrlTaskControlBlock),

  .stack_mem = &MotorCtrlTaskBuffer[0],

  .stack_size = sizeof(MotorCtrlTaskBuffer),

  .priority = (osPriority_t) osPriorityHigh,

};

/* Definitions for CommTask */

osThreadId_t CommTaskHandle;

uint32_t CommTaskBuffer[ 512 ];

osStaticThreadDef_t CommTaskControlBlock;

const osThreadAttr_t CommTask_attributes = {

  .name = "CommTask",

  .cb_mem = &CommTaskControlBlock,

  .cb_size = sizeof(CommTaskControlBlock),

  .stack_mem = &CommTaskBuffer[0],

  .stack_size = sizeof(CommTaskBuffer),

  .priority = (osPriority_t) osPriorityAboveNormal,

};

/* Definitions for ImuTask */

osThreadId_t ImuTaskHandle;

uint32_t ImuTaskBuffer[ 384 ];

osStaticThreadDef_t ImuTaskControlBlock;

const osThreadAttr_t ImuTask_attributes = {

  .name = "ImuTask",

  .cb_mem = &ImuTaskControlBlock,

  .cb_size = sizeof(ImuTaskControlBlock),

  .stack_mem = &ImuTaskBuffer[0],

  .stack_size = sizeof(ImuTaskBuffer),

  .priority = (osPriority_t) osPriorityNormal,

};

/* Definitions for TelemetryTask */

osThreadId_t TelemetryTaskHandle;

uint32_t TelemetryTaskBuffer[ 384 ];

osStaticThreadDef_t TelemetryTaskControlBlock;

const osThreadAttr_t TelemetryTask_attributes = {

  .name = "TelemetryTask",

  .cb_mem = &TelemetryTaskControlBlock,

  .cb_size = sizeof(TelemetryTaskControlBlock),

  .stack_mem = &TelemetryTaskBuffer[0],

  .stack_size = sizeof(TelemetryTaskBuffer),

  .priority = (osPriority_t) osPriorityBelowNormal,

};

/* Definitions for DiagTask */

osThreadId_t DiagTaskHandle;

uint32_t DiagTaskBuffer[ 256 ];

osStaticThreadDef_t DiagTaskControlBlock;

const osThreadAttr_t DiagTask_attributes = {

  .name = "DiagTask",

  .cb_mem = &DiagTaskControlBlock,

  .cb_size = sizeof(DiagTaskControlBlock),

  .stack_mem = &DiagTaskBuffer[0],

  .stack_size = sizeof(DiagTaskBuffer),

  .priority = (osPriority_t) osPriorityLow,

};

/* Definitions for DriveCmdQueue */

osMessageQueueId_t DriveCmdQueueHandle;

uint8_t DriveCmdQueueBuffer[ 4 * sizeof( DriveCommand_t ) ];

osStaticMessageQDef_t DriveCmdQueueControlBlock;

const osMessageQueueAttr_t DriveCmdQueue_attributes = {

  .name = "DriveCmdQueue",

  .cb_mem = &DriveCmdQueueControlBlock,

  .cb_size = sizeof(DriveCmdQueueControlBlock),

  .mq_mem = &DriveCmdQueueBuffer,

  .mq_size = sizeof(DriveCmdQueueBuffer)

};

/* Definitions for UsbRxQueue */

osMessageQueueId_t UsbRxQueueHandle;

uint8_t UsbRxQueueBuffer[ 8 * sizeof( UsbRxChunk_t ) ];

osStaticMessageQDef_t UsbRxQueueControlBlock;

const osMessageQueueAttr_t UsbRxQueue_attributes = {

  .name = "UsbRxQueue",

  .cb_mem = &UsbRxQueueControlBlock,

  .cb_size = sizeof(UsbRxQueueControlBlock),

  .mq_mem = &UsbRxQueueBuffer,

  .mq_size = sizeof(UsbRxQueueBuffer)

};

/* Definitions for TelemetryQueue */

osMessageQueueId_t TelemetryQueueHandle;

uint8_t TelemetryQueueBuffer[ 4 * sizeof( TelemetryData_t ) ];

osStaticMessageQDef_t TelemetryQueueControlBlock;

const osMessageQueueAttr_t TelemetryQueue_attributes = {

  .name = "TelemetryQueue",

  .cb_mem = &TelemetryQueueControlBlock,

  .cb_size = sizeof(TelemetryQueueControlBlock),

  .mq_mem = &TelemetryQueueBuffer,

  .mq_size = sizeof(TelemetryQueueBuffer)

};

/* Definitions for SystemEvents */

osEventFlagsId_t SystemEventsHandle;

osStaticEventGroupDef_t SystemEventsControlBlock;

const osEventFlagsAttr_t SystemEvents_attributes = {

  .name = "SystemEvents",

  .cb_mem = &SystemEventsControlBlock,

  .cb_size = sizeof(SystemEventsControlBlock),

};

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartSafetyTask(void *argument);

void StartMotorCtrlTask(void *argument);

void StartCommTask(void *argument);

void StartImuTask(void *argument);

void StartTelemetryTask(void *argument);

void StartDiagTask(void *argument);

extern void MX_USB_DEVICE_Init(void);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)

{

   /* Run time stack overflow checking is performed if

   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is

   called if a stack overflow is detected. */

}

/* USER CODE END 4 */

/* USER CODE BEGIN 5 */

void vApplicationMallocFailedHook(void)

{

   /* vApplicationMallocFailedHook() will only be called if

   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook

   function that will get called if a call to pvPortMalloc() fails.

   pvPortMalloc() is called internally by the kernel whenever a task, queue,

   timer or semaphore is created. It is also called by various parts of the

   demo application. If heap_1.c or heap_2.c are used, then the size of the

   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in

   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used

   to query the size of free heap space that remains (although it does not

   provide information on how the remaining heap might be fragmented). */

}

/* USER CODE END 5 */

/**

  * @brief  FreeRTOS initialization

  * @param  None

  * @retval None

  */

void MX_FREERTOS_Init(void) {

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */

  /* add mutexes, ... */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  /* add semaphores, ... */

  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */

  /* start timers, add new ones, ... */

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */

  /* creation of DriveCmdQueue */

  DriveCmdQueueHandle = osMessageQueueNew (4, sizeof(DriveCommand_t), &DriveCmdQueue_attributes);

  /* creation of UsbRxQueue */

  UsbRxQueueHandle = osMessageQueueNew (8, sizeof(UsbRxChunk_t), &UsbRxQueue_attributes);

  /* creation of TelemetryQueue */

  TelemetryQueueHandle = osMessageQueueNew (4, sizeof(TelemetryData_t), &TelemetryQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */

  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */

  /* creation of SafetyTask */

  SafetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &SafetyTask_attributes);

  /* creation of MotorCtrlTask */

  MotorCtrlTaskHandle = osThreadNew(StartMotorCtrlTask, NULL, &MotorCtrlTask_attributes);

  /* creation of CommTask */

  CommTaskHandle = osThreadNew(StartCommTask, NULL, &CommTask_attributes);

  /* creation of ImuTask */

  ImuTaskHandle = osThreadNew(StartImuTask, NULL, &ImuTask_attributes);

  /* creation of TelemetryTask */

  TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &TelemetryTask_attributes);

  /* creation of DiagTask */

  DiagTaskHandle = osThreadNew(StartDiagTask, NULL, &DiagTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* add threads, ... */

  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */

  /* creation of SystemEvents */

  SystemEventsHandle = osEventFlagsNew(&SystemEvents_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */

  /* add events, ... */

  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartSafetyTask */

/**

  * @brief  Function implementing the SafetyTask thread.

  * @param  argument: Not used

  * @retval None

  */

/* USER CODE END Header_StartSafetyTask */

void StartSafetyTask(void *argument)

{

  /* init code for USB_DEVICE */

  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartSafetyTask */

    if (!DriveMotor_Init())

    {

        (void)osEventFlagsSet(SystemEventsHandle, AMR_EVT_FAULT);

    }

    for (;;)

    {

        uint32_t flags = osEventFlagsGet(SystemEventsHandle);

        if ((flags & (AMR_EVT_MOTOR_ENABLED |

                      AMR_EVT_FAULT |

                      AMR_EVT_ESTOP)) != AMR_EVT_MOTOR_ENABLED)

        {

            DriveMotor_Stop();

        }

        osDelay(5);

    }

  /* USER CODE END StartSafetyTask */

}

/* USER CODE BEGIN Header_StartMotorCtrlTask */

/**

* @brief Function implementing the MotorCtrlTask thread.

* @param argument: Not used

* @retval None

*/

/* USER CODE END Header_StartMotorCtrlTask */

void StartMotorCtrlTask(void *argument)

{

  /* USER CODE BEGIN StartMotorCtrlTask */

    DriveEncoderSample_t encoder = {0};

    DriveKinematicsSample_t kinematics = {0};

    DriveSpeedControlOutput_t control_output = {0};

    DriveMotionProfileState_t motion_profile_state = {0};

    DriveMotionProfileOutput_t motion_profile_output = {0};

    ImuHeadingHoldState_t heading_hold_state = {0};

    ImuHeadingHoldOutput_t heading_hold_output = {0};

    TurnControllerState_t turn_state = {0};

    TurnControllerOutput_t turn_output = {0};

    DistanceControllerState_t distance_state = {0};

    DistanceControllerOutput_t distance_output = {0};

    DriveOdometryState_t odometry_state = {0};

    DriveOdometryOutput_t odometry_output = {0};

    DriveCommand_t active_command = {0};

    uint32_t next_wake = osKernelGetTickCount();

    bool command_active = false;

    bool kinematics_valid = false;

    /* V3.2 normal-command stop handoff state. */
    bool normal_stop_brake_active = false;
    uint32_t normal_stop_brake_started_ms = 0U;
    uint32_t normal_stop_brake_settle_started_ms = 0U;

    DriveKinematics_Init();

    DriveSpeedControl_Init();

    DriveMotionProfile_Init(&motion_profile_state);

    ImuHeadingHold_Init(&heading_hold_state);

    TurnController_Init(&turn_state);

    DistanceController_Init(&distance_state);

    DriveOdometry_Init(&odometry_state);

    if (!DriveEncoder_Init())

    {

        (void)osEventFlagsSet(SystemEventsHandle, AMR_EVT_FAULT);

    }

    for (;;)

    {

        const uint32_t now_ms = osKernelGetTickCount();

        if (DriveEncoder_Sample(&encoder))

        {

            g_encoder_left_count = encoder.left_total_count;

            g_encoder_right_count = encoder.right_total_count;

            g_encoder_left_delta = encoder.left_delta_count;

            g_encoder_right_delta = encoder.right_delta_count;

            if (DriveKinematics_Update(&encoder, &kinematics))

            {

                kinematics_valid = true;

                g_wheel_left_position_m = kinematics.left_position_m;

                g_wheel_right_position_m = kinematics.right_position_m;

                g_wheel_left_speed_m_s = kinematics.left_speed_m_s;

                g_wheel_right_speed_m_s = kinematics.right_speed_m_s;

                g_wheel_left_speed_raw_m_s =

                    kinematics.left_speed_raw_m_s;

                g_wheel_right_speed_raw_m_s =

                    kinematics.right_speed_raw_m_s;

                g_wheel_left_rpm = kinematics.left_speed_rpm;

                g_wheel_right_rpm = kinematics.right_speed_rpm;

                g_wheel_left_speed_rad_s =

                    kinematics.left_speed_rad_s;

                g_wheel_right_speed_rad_s =

                    kinematics.right_speed_rad_s;

                const bool odometry_updated =

                    DriveOdometry_Update(

                        &odometry_state,

                        (g_imu_ready != 0U),

                        kinematics.left_position_m,

                        kinematics.right_position_m,

                        kinematics.left_speed_m_s,

                        kinematics.right_speed_m_s,

                        g_imu_yaw_rad,

                        g_imu_gyro_z_rad_s,

                        now_ms,

                        &odometry_output);

                g_odom_ready =

                    (odometry_updated && odometry_output.valid) ? 1U : 0U;

                g_odom_x_m = odometry_output.x_m;

                g_odom_y_m = odometry_output.y_m;

                g_odom_yaw_rad = odometry_output.yaw_rad;

                g_odom_linear_velocity_m_s =

                    odometry_output.linear_velocity_m_s;

                g_odom_angular_velocity_rad_s =

                    odometry_output.angular_velocity_rad_s;

                g_odom_sample_count = odometry_output.sample_count;

                g_odom_reset_count = odometry_state.reset_count;

                if (!odometry_updated && (g_imu_ready != 0U))

                {

                    ++g_odom_error_count;

                }

            }

        }

        /* Queue chi giu lenh moi nhat; doc het neu co hon mot lenh. */

        DriveCommand_t received_command;

        DriveCommandPolicy_t command_policy =

        {

            .preserve_speed_controller = false,

            .reset_heading_hold = true

        };

        bool received_new_command = false;

        while (osMessageQueueGet(DriveCmdQueueHandle,

                                 &received_command,

                                 NULL,

                                 0U) == osOK)

        {

            command_policy = DriveCommandPolicy_Evaluate(

                command_active,

                &active_command,

                &received_command);

            active_command = received_command;

            received_new_command = true;

        }

        const bool received_zero_wheel_command =
            received_new_command &&
            (active_command.command_type == AMR_MOTION_COMMAND_WHEEL_SPEED) &&
            (active_command.left_target_mrad_s == 0) &&
            (active_command.right_target_mrad_s == 0);

        if (received_new_command && !received_zero_wheel_command)
        {
            normal_stop_brake_active = false;
            normal_stop_brake_settle_started_ms = 0U;
        }

        if (received_new_command)

        {

            /*
             * V3.2: preserve wheel-controller output state for an explicit
             * normal 0/0 stop so the active-brake branch can slew from the
             * actual last PWM instead of being reset to zero first.
             */
            if (command_policy.preserve_speed_controller ||
                received_zero_wheel_command)

            {

                ++g_drive_command_refresh_count;

                if (command_policy.reset_heading_hold)

                {

                    DriveMotionProfile_Reset(&motion_profile_state);

                    ImuHeadingHold_Reset(&heading_hold_state);

                }

            }

            else

            {

                DriveSpeedControl_Reset();

                DriveMotionProfile_Reset(&motion_profile_state);

                ImuHeadingHold_Reset(&heading_hold_state);

                TurnController_Cancel(&turn_state);

                DistanceController_Cancel(&distance_state);

                ++g_drive_controller_reset_count;

                g_motion_profile_status = DRIVE_MOTION_PROFILE_IDLE;

                g_motion_profile_fault = 0U;

                g_turn_active = 0U;

                g_turn_status = TURN_CONTROLLER_IDLE;

                g_turn_target_angle_rad = 0.0f;

                g_turn_traveled_angle_rad = 0.0f;

                g_turn_error_rad = 0.0f;

                g_turn_yaw_rate_command_rad_s = 0.0f;

                g_distance_active = 0U;

                g_distance_status = DISTANCE_CONTROLLER_IDLE;

                g_distance_target_m = 0.0f;

                g_distance_traveled_m = 0.0f;

                g_distance_remaining_m = 0.0f;

                g_distance_speed_command_m_s = 0.0f;

            }

            if (active_command.command_type ==

                AMR_MOTION_COMMAND_TURN_RELATIVE)

            {

                const bool turn_started =

                    (g_imu_ready != 0U) &&

                    TurnController_Start(

                        &turn_state,

                        (float)active_command.turn_angle_mrad * 0.001f,

                        (float)active_command.turn_max_yaw_rate_mrad_s *

                            0.001f,

                        (uint32_t)active_command.timeout_ms,

                        g_imu_yaw_rad,

                        now_ms);

                if (!turn_started)

                {

                    DriveMotor_Stop();

                    command_active = false;

                    g_drive_command_active = 0U;

                    g_turn_active = 0U;

                    g_turn_status = TURN_CONTROLLER_FAULT_IMU;

                    ++g_turn_imu_fault_count;

                    (void)osEventFlagsClear(

                        SystemEventsHandle,

                        AMR_EVT_CMD_VALID | AMR_EVT_MOTOR_ENABLED);

                    (void)osEventFlagsSet(SystemEventsHandle,

                                          AMR_EVT_FAULT);

                }

                else

                {

                    command_active = true;

                    g_drive_command_active = 1U;

                    g_turn_active = 1U;

                    g_turn_status = TURN_CONTROLLER_RUNNING;

                    g_turn_target_angle_rad =

                        (float)active_command.turn_angle_mrad * 0.001f;

                    g_turn_traveled_angle_rad = 0.0f;

                    g_turn_error_rad = g_turn_target_angle_rad;

                    g_turn_yaw_rate_command_rad_s = 0.0f;

                }

            }

            else if (active_command.command_type ==

                     AMR_MOTION_COMMAND_DRIVE_DISTANCE)

            {

                const bool distance_started =

                    kinematics_valid &&

                    (g_imu_ready != 0U) &&

                    DistanceController_Start(

                        &distance_state,

                        (float)active_command.distance_mm * 0.001f,

                        (float)active_command.distance_max_speed_mm_s *

                            0.001f,

                        (uint32_t)active_command.timeout_ms,

                        kinematics.left_position_m,

                        kinematics.right_position_m,

                        now_ms);

                if (!distance_started)

                {

                    DriveMotor_Stop();

                    command_active = false;

                    g_drive_command_active = 0U;

                    g_distance_active = 0U;

                    g_distance_status =

                        (kinematics_valid && (g_imu_ready != 0U)) ?

                        (uint8_t)distance_state.status :

                        (uint8_t)DISTANCE_CONTROLLER_FAULT_MEASUREMENT;

                    ++g_distance_measurement_fault_count;

                    (void)osEventFlagsClear(

                        SystemEventsHandle,

                        AMR_EVT_CMD_VALID | AMR_EVT_MOTOR_ENABLED);

                    (void)osEventFlagsSet(SystemEventsHandle,

                                          AMR_EVT_FAULT);

                }

                else

                {

                    command_active = true;

                    g_drive_command_active = 1U;

                    g_distance_active = 1U;

                    g_distance_status = DISTANCE_CONTROLLER_RUNNING;

                    g_distance_target_m =

                        (float)active_command.distance_mm * 0.001f;

                    g_distance_traveled_m = 0.0f;

                    g_distance_remaining_m = g_distance_target_m;

                    g_distance_speed_command_m_s = 0.0f;

                }

            }

            else

            {

                command_active = true;

                g_drive_command_active = 1U;

            }

        }

        const uint32_t flags = osEventFlagsGet(SystemEventsHandle);

        if (command_active)

        {

            uint32_t command_age_ms =

                now_ms - active_command.received_at_ms;

            g_drive_command_age_ms = command_age_ms;

            if ((active_command.command_type ==

                 AMR_MOTION_COMMAND_WHEEL_SPEED) &&

                ((active_command.left_target_mrad_s != 0) ||
                 (active_command.right_target_mrad_s != 0)) &&

                (command_age_ms >

                 (uint32_t)active_command.timeout_ms))

            {

                /* Mat lenh: dung tai MotorCtrlTask va latch FAULT. */

                DriveMotor_Stop();

                DriveSpeedControl_Reset();

                normal_stop_brake_active = false;
                normal_stop_brake_settle_started_ms = 0U;

                DriveMotionProfile_Reset(&motion_profile_state);

                ImuHeadingHold_Reset(&heading_hold_state);

                TurnController_Cancel(&turn_state);

                DistanceController_Cancel(&distance_state);

                command_active = false;

                g_drive_command_active = 0U;

                g_drive_left_pwm = 0;

                g_drive_right_pwm = 0;

                g_drive_target_left_rad_s = 0.0f;

                g_drive_target_right_rad_s = 0.0f;

                g_heading_hold_active = 0U;

                g_heading_reference_rad = 0.0f;

                g_heading_error_rad = 0.0f;

                g_heading_correction_rad_s = 0.0f;

                g_turn_active = 0U;

                g_turn_status = TURN_CONTROLLER_IDLE;

                g_distance_active = 0U;

                g_distance_status = DISTANCE_CONTROLLER_IDLE;

                ++g_drive_timeout_count;

                (void)osEventFlagsClear(

                    SystemEventsHandle,

                    AMR_EVT_CMD_VALID | AMR_EVT_MOTOR_ENABLED);

                (void)osEventFlagsSet(SystemEventsHandle,

                                      AMR_EVT_FAULT);

            }

            else if (((flags & (AMR_EVT_FAULT | AMR_EVT_ESTOP)) != 0U) ||

                     ((flags & AMR_EVT_MOTOR_ENABLED) == 0U))

            {

                DriveMotor_Stop();

                DriveSpeedControl_Reset();

                normal_stop_brake_active = false;
                normal_stop_brake_settle_started_ms = 0U;

                DriveMotionProfile_Reset(&motion_profile_state);

                ImuHeadingHold_Reset(&heading_hold_state);

                TurnController_Cancel(&turn_state);

                DistanceController_Cancel(&distance_state);

                command_active = false;

                g_drive_command_active = 0U;

                g_drive_left_pwm = 0;

                g_drive_right_pwm = 0;

                g_heading_hold_active = 0U;

                g_heading_reference_rad = 0.0f;

                g_heading_error_rad = 0.0f;

                g_heading_correction_rad_s = 0.0f;

                g_turn_active = 0U;

                g_turn_status = TURN_CONTROLLER_IDLE;

                g_distance_active = 0U;

                g_distance_status = DISTANCE_CONTROLLER_IDLE;

            }

            else if ((active_command.command_type ==

                      AMR_MOTION_COMMAND_WHEEL_SPEED) &&

                     (active_command.left_target_mrad_s == 0) &&

                     (active_command.right_target_mrad_s == 0))

            {

                /*
                 * V3.2 NORMAL STOP ACTIVE BRAKE
                 *
                 * Explicit WHEEL_SPEED 0/0 is a deliberate stop, not a fault.
                 * Keep motor authority active long enough for the V3
                 * drive-speed controller to apply bounded reverse torque from
                 * measured wheel speed. Only after both measured wheel speeds remain settled
                 * continuously (or the bounded handoff timeout expires) do we
                 * release MOTOR_ENABLED and hard-stop the bridge.
                 *
                 * FAULT / ESTOP / motor-disable is handled by the branch above
                 * and remains an immediate DriveMotor_Stop().
                 */
                DriveMotionProfile_Reset(&motion_profile_state);

                ImuHeadingHold_Reset(&heading_hold_state);

                TurnController_Cancel(&turn_state);

                DistanceController_Cancel(&distance_state);

                g_motion_profile_status = DRIVE_MOTION_PROFILE_IDLE;

                g_motion_profile_fault = 0U;

                g_heading_hold_active = 0U;

                g_heading_reference_rad = 0.0f;

                g_heading_error_rad = 0.0f;

                g_heading_correction_rad_s = 0.0f;

                g_turn_active = 0U;

                g_turn_status = TURN_CONTROLLER_IDLE;

                g_distance_active = 0U;

                g_distance_status = DISTANCE_CONTROLLER_IDLE;

                g_drive_target_left_rad_s = 0.0f;

                g_drive_target_right_rad_s = 0.0f;

                if (!kinematics_valid)
                {
                    /*
                     * No trustworthy wheel-speed feedback: do not attempt
                     * active braking. Fall back to the original hard stop.
                     */
                    DriveMotor_Stop();

                    DriveSpeedControl_Reset();

                    normal_stop_brake_active = false;
                    normal_stop_brake_settle_started_ms = 0U;

                    command_active = false;

                    g_drive_command_active = 0U;

                    g_drive_left_pwm = 0;

                    g_drive_right_pwm = 0;

                    g_drive_error_left_rad_s = 0.0f;

                    g_drive_error_right_rad_s = 0.0f;

                    (void)osEventFlagsClear(

                        SystemEventsHandle,

                        AMR_EVT_CMD_VALID | AMR_EVT_MOTOR_ENABLED);
                }
                else
                {
                    if (!normal_stop_brake_active)
                    {
                        normal_stop_brake_active = true;
                        normal_stop_brake_started_ms = now_ms;
                        normal_stop_brake_settle_started_ms = 0U;
                    }

                    DriveSpeedControl_Update(
                        0.0f,
                        0.0f,
                        kinematics.left_speed_rad_s,
                        kinematics.right_speed_rad_s,
                        &control_output);

                    g_drive_target_left_rad_s =
                        control_output.left_target_rad_s;

                    g_drive_target_right_rad_s =
                        control_output.right_target_rad_s;

                    g_drive_error_left_rad_s =
                        control_output.left_error_rad_s;

                    g_drive_error_right_rad_s =
                        control_output.right_error_rad_s;

                    g_drive_left_pwm = control_output.left_pwm;

                    g_drive_right_pwm = control_output.right_pwm;

                    DriveMotor_Set(control_output.left_pwm,
                                   control_output.right_pwm);

                    /*
                     * V3.2 completion authority comes from measured wheel speed,
                     * not from PWM==0.  A one-cycle zero controller output must
                     * never be interpreted as "the chassis is stopped" while
                     * the encoders still report wheel motion.  Require both
                     * wheels to stay below the settle threshold continuously.
                     */
                    const bool normal_stop_wheels_below_settle =
                        (kinematics.left_speed_rad_s <=
                         DRIVE_NORMAL_STOP_SETTLE_RAD_S) &&
                        (kinematics.left_speed_rad_s >=
                         -DRIVE_NORMAL_STOP_SETTLE_RAD_S) &&
                        (kinematics.right_speed_rad_s <=
                         DRIVE_NORMAL_STOP_SETTLE_RAD_S) &&
                        (kinematics.right_speed_rad_s >=
                         -DRIVE_NORMAL_STOP_SETTLE_RAD_S);

                    if (normal_stop_wheels_below_settle)
                    {
                        if (normal_stop_brake_settle_started_ms == 0U)
                        {
                            normal_stop_brake_settle_started_ms = now_ms;
                        }
                    }
                    else
                    {
                        normal_stop_brake_settle_started_ms = 0U;
                    }

                    const bool normal_stop_brake_settled =
                        (normal_stop_brake_settle_started_ms != 0U) &&
                        ((uint32_t)(now_ms -
                         normal_stop_brake_settle_started_ms) >=
                         DRIVE_NORMAL_STOP_SETTLE_HOLD_MS);

                    const bool normal_stop_brake_timed_out =
                        ((uint32_t)(now_ms -
                         normal_stop_brake_started_ms) >=
                         DRIVE_NORMAL_STOP_BRAKE_TIMEOUT_MS);

                    if (normal_stop_brake_settled ||
                        normal_stop_brake_timed_out)
                    {
                        DriveMotor_Stop();

                        DriveSpeedControl_Reset();

                        normal_stop_brake_active = false;
                        normal_stop_brake_settle_started_ms = 0U;

                        command_active = false;

                        g_drive_command_active = 0U;

                        g_drive_left_pwm = 0;

                        g_drive_right_pwm = 0;

                        g_drive_error_left_rad_s = 0.0f;

                        g_drive_error_right_rad_s = 0.0f;

                        (void)osEventFlagsClear(

                            SystemEventsHandle,

                            AMR_EVT_CMD_VALID | AMR_EVT_MOTOR_ENABLED);
                    }
                    else
                    {
                        /*
                         * Keep SafetyTask's normal motor-enable condition valid
                         * during this short, bounded active-brake handoff.
                         */
                        command_active = true;
                        g_drive_command_active = 1U;
                    }
                }

            }

            else if (kinematics_valid)

            {

                float left_target_rad_s = 0.0f;

                float right_target_rad_s = 0.0f;

                if (active_command.command_type ==

                    AMR_MOTION_COMMAND_TURN_RELATIVE)

                {

                    DriveMotionProfile_Reset(&motion_profile_state);

                    g_motion_profile_status = DRIVE_MOTION_PROFILE_IDLE;

                    g_motion_profile_fault = 0U;

                    ImuHeadingHold_Reset(&heading_hold_state);

                    g_heading_hold_active = 0U;

                    g_heading_reference_rad = 0.0f;

                    g_heading_error_rad = 0.0f;

                    g_heading_correction_rad_s = 0.0f;

                    TurnController_Update(

                        &turn_state,

                        (g_imu_ready != 0U),

                        g_imu_yaw_rad,

                        g_imu_gyro_z_rad_s,

                        DRIVE_CONTROL_PERIOD_S,

                        now_ms,

                        &turn_output);

                    g_turn_status = (uint8_t)turn_output.status;

                    g_turn_active = turn_output.active ? 1U : 0U;

                    g_turn_target_angle_rad =

                        turn_output.target_relative_angle_rad;

                    g_turn_traveled_angle_rad =

                        turn_output.traveled_angle_rad;

                    g_turn_error_rad = turn_output.error_rad;

                    g_turn_yaw_rate_command_rad_s =

                        turn_output.yaw_rate_command_rad_s;

                    if (turn_output.completed)

                    {

                        DriveMotor_Stop();

                        DriveSpeedControl_Reset();

                        command_active = false;

                        g_drive_command_active = 0U;

                        g_drive_left_pwm = 0;

                        g_drive_right_pwm = 0;

                        g_drive_target_left_rad_s = 0.0f;

                        g_drive_target_right_rad_s = 0.0f;

                        ++g_turn_complete_count;

                        (void)osEventFlagsClear(

                            SystemEventsHandle,

                            AMR_EVT_CMD_VALID | AMR_EVT_MOTOR_ENABLED);

                    }

                    else if (turn_output.fault)

                    {

                        DriveMotor_Stop();

                        DriveSpeedControl_Reset();

                        command_active = false;

                        g_drive_command_active = 0U;

                        g_drive_left_pwm = 0;

                        g_drive_right_pwm = 0;

                        g_drive_target_left_rad_s = 0.0f;

                        g_drive_target_right_rad_s = 0.0f;

                        if (turn_output.status ==

                            TURN_CONTROLLER_FAULT_TIMEOUT)

                        {

                            ++g_turn_timeout_count;

                        }

                        else

                        {

                            ++g_turn_imu_fault_count;

                        }

                        (void)osEventFlagsClear(

                            SystemEventsHandle,

                            AMR_EVT_CMD_VALID | AMR_EVT_MOTOR_ENABLED);

                        (void)osEventFlagsSet(SystemEventsHandle,

                                              AMR_EVT_FAULT);

                    }

                    else

                    {

                        left_target_rad_s =

                            turn_output.left_target_rad_s;

                        right_target_rad_s =

                            turn_output.right_target_rad_s;

                    }

                }

                else if (active_command.command_type ==

                         AMR_MOTION_COMMAND_DRIVE_DISTANCE)

                {

                    if (g_imu_ready == 0U)

                    {

                        distance_state.status =

                            DISTANCE_CONTROLLER_FAULT_MEASUREMENT;

                        distance_state.linear_speed_command_m_s = 0.0f;

                    }

                    DistanceController_Update(

                        &distance_state,

                        kinematics.left_position_m,

                        kinematics.right_position_m,

                        kinematics.left_speed_m_s,

                        kinematics.right_speed_m_s,

                        DRIVE_CONTROL_PERIOD_S,

                        now_ms,

                        &distance_output);

                    g_distance_status = (uint8_t)distance_output.status;

                    g_distance_active = distance_output.active ? 1U : 0U;

                    g_distance_target_m =

                        distance_output.target_distance_m;

                    g_distance_traveled_m =

                        distance_output.traveled_distance_m;

                    g_distance_remaining_m =

                        distance_output.remaining_distance_m;

                    g_distance_speed_command_m_s =

                        distance_output.linear_speed_command_m_s;

                    if (distance_output.completed)

                    {

                        DriveMotor_Stop();

                        DriveSpeedControl_Reset();

                        ImuHeadingHold_Reset(&heading_hold_state);

                        command_active = false;

                        g_drive_command_active = 0U;

                        g_drive_left_pwm = 0;

                        g_drive_right_pwm = 0;

                        g_drive_target_left_rad_s = 0.0f;

                        g_drive_target_right_rad_s = 0.0f;

                        g_heading_hold_active = 0U;

                        ++g_distance_complete_count;

                        (void)osEventFlagsClear(

                            SystemEventsHandle,

                            AMR_EVT_CMD_VALID | AMR_EVT_MOTOR_ENABLED);

                    }

                    else if (distance_output.fault)

                    {

                        DriveMotor_Stop();

                        DriveSpeedControl_Reset();

                        ImuHeadingHold_Reset(&heading_hold_state);

                        command_active = false;

                        g_drive_command_active = 0U;

                        g_drive_left_pwm = 0;

                        g_drive_right_pwm = 0;

                        g_drive_target_left_rad_s = 0.0f;

                        g_drive_target_right_rad_s = 0.0f;

                        g_heading_hold_active = 0U;

                        if (distance_output.status ==

                            DISTANCE_CONTROLLER_FAULT_TIMEOUT)

                        {

                            ++g_distance_timeout_count;

                        }

                        else

                        {

                            ++g_distance_measurement_fault_count;

                        }

                        (void)osEventFlagsClear(

                            SystemEventsHandle,

                            AMR_EVT_CMD_VALID | AMR_EVT_MOTOR_ENABLED);

                        (void)osEventFlagsSet(SystemEventsHandle,

                                              AMR_EVT_FAULT);

                    }

                    else

                    {

                        const float wheel_radius_m =

                            DRIVE_WHEEL_DIAMETER_M * 0.5f;

                        const float common_wheel_target_rad_s =

                            distance_output.linear_speed_command_m_s /

                            wheel_radius_m;

                        ImuHeadingHold_Update(

                            &heading_hold_state,

                            (g_imu_ready != 0U),

                            g_imu_yaw_rad,

                            g_imu_gyro_z_rad_s,

                            common_wheel_target_rad_s,

                            common_wheel_target_rad_s,

                            DRIVE_CONTROL_TARGET_LIMIT_RAD_S,

                            DRIVE_CONTROL_PERIOD_S,

                            &heading_hold_output);

                        g_heading_hold_active =

                            heading_hold_output.active ? 1U : 0U;

                        g_heading_reference_rad =

                            heading_hold_output.reference_yaw_rad;

                        g_heading_error_rad =

                            heading_hold_output.error_rad;

                        g_heading_correction_rad_s =

                            heading_hold_output.correction_rad_s;

                        g_heading_desired_yaw_rate_rad_s =

                            heading_hold_output.desired_yaw_rate_rad_s;

                        g_heading_yaw_rate_error_rad_s =

                            heading_hold_output.yaw_rate_error_rad_s;

                        g_heading_rate_ff_correction_rad_s =

                            heading_hold_output.rate_ff_correction_rad_s;

                        g_heading_rate_p_correction_rad_s =

                            heading_hold_output.rate_p_correction_rad_s;

                        g_heading_rate_i_correction_rad_s =

                            heading_hold_output.rate_i_correction_rad_s;

                        g_heading_controller_version =

                            heading_hold_output.controller_version;

                        g_heading_correction_saturated =

                            heading_hold_output.correction_saturated ?

                            1U : 0U;

                        left_target_rad_s =

                            heading_hold_output.left_target_rad_s;

                        right_target_rad_s =

                            heading_hold_output.right_target_rad_s;

                    }

                }

                else

                {

                    const float requested_left_rad_s =

                        (float)active_command.left_target_mrad_s * 0.001f;

                    const float requested_right_rad_s =

                        (float)active_command.right_target_mrad_s * 0.001f;

                    /*

                     * AUTO WHEEL_SPEED is the ROS trajectory-control path.

                     * In this mode ROS owns yaw continuously.  Do not place

                     * another heading/caster controller between ROS and the

                     * wheel-speed PI because both legacy blocks use the same

                     * small wheel-difference threshold and can create a hidden

                     * authority handoff / steering discontinuity.

                     *

                     * MANUAL WHEEL_SPEED keeps the commissioned legacy

                     * motion-profile + heading-hold behavior.  TURN_RELATIVE

                     * and DRIVE_DISTANCE are handled by their own branches

                     * above and are intentionally unchanged.

                     */

                    if (AppComm_GetMode() == AMR_MODE_AUTO)

                    {

                        DriveMotionProfile_Reset(&motion_profile_state);

                        g_motion_profile_status = DRIVE_MOTION_PROFILE_IDLE;

                        g_motion_profile_fault = 0U;

                        ImuHeadingHold_Reset(&heading_hold_state);

                        g_heading_hold_active = 0U;

                        g_heading_reference_rad = 0.0f;

                        g_heading_error_rad = 0.0f;

                        g_heading_correction_rad_s = 0.0f;

                        g_heading_desired_yaw_rate_rad_s = 0.0f;

                        g_heading_yaw_rate_error_rad_s = 0.0f;

                        g_heading_rate_ff_correction_rad_s = 0.0f;

                        g_heading_rate_p_correction_rad_s = 0.0f;

                        g_heading_rate_i_correction_rad_s = 0.0f;

                        g_heading_correction_saturated = 0U;

                        left_target_rad_s = requested_left_rad_s;

                        right_target_rad_s = requested_right_rad_s;

                    }

                    else

                    {

                        DriveMotionProfile_Update(

                            &motion_profile_state,

                            (g_imu_ready != 0U),

                            g_imu_yaw_rad,

                            g_imu_gyro_z_rad_s,

                            requested_left_rad_s,

                            requested_right_rad_s,

                            DRIVE_CONTROL_PERIOD_S,

                            now_ms,

                            &motion_profile_output);

                        g_motion_profile_status =

                            motion_profile_output.status;

                        g_motion_profile_fault =

                            motion_profile_output.fault ? 1U : 0U;

                        ImuHeadingHold_Update(

                            &heading_hold_state,

                            (g_imu_ready != 0U),

                            g_imu_yaw_rad,

                            g_imu_gyro_z_rad_s,

                            motion_profile_output.left_target_rad_s,

                            motion_profile_output.right_target_rad_s,

                            DRIVE_CONTROL_TARGET_LIMIT_RAD_S,

                            DRIVE_CONTROL_PERIOD_S,

                            &heading_hold_output);

                        g_heading_hold_active =

                            heading_hold_output.active ? 1U : 0U;

                        g_heading_reference_rad =

                            heading_hold_output.reference_yaw_rad;

                        g_heading_error_rad =

                            heading_hold_output.error_rad;

                        g_heading_correction_rad_s =

                            heading_hold_output.correction_rad_s;

                        g_heading_desired_yaw_rate_rad_s =

                            heading_hold_output.desired_yaw_rate_rad_s;

                        g_heading_yaw_rate_error_rad_s =

                            heading_hold_output.yaw_rate_error_rad_s;

                        g_heading_rate_ff_correction_rad_s =

                            heading_hold_output.rate_ff_correction_rad_s;

                        g_heading_rate_p_correction_rad_s =

                            heading_hold_output.rate_p_correction_rad_s;

                        g_heading_rate_i_correction_rad_s =

                            heading_hold_output.rate_i_correction_rad_s;

                        g_heading_controller_version =

                            heading_hold_output.controller_version;

                        g_heading_correction_saturated =

                            heading_hold_output.correction_saturated ?

                            1U : 0U;

                        left_target_rad_s =

                            heading_hold_output.left_target_rad_s;

                        right_target_rad_s =

                            heading_hold_output.right_target_rad_s;

                    }

                }

                if (command_active)

                {

                    DriveSpeedControl_Update(

                        left_target_rad_s,

                        right_target_rad_s,

                        kinematics.left_speed_rad_s,

                        kinematics.right_speed_rad_s,

                        &control_output);

                    g_drive_target_left_rad_s =

                        control_output.left_target_rad_s;

                    g_drive_target_right_rad_s =

                        control_output.right_target_rad_s;

                    g_drive_error_left_rad_s =

                        control_output.left_error_rad_s;

                    g_drive_error_right_rad_s =

                        control_output.right_error_rad_s;

                    g_drive_left_pwm = control_output.left_pwm;

                    g_drive_right_pwm = control_output.right_pwm;

                    DriveMotor_Set(control_output.left_pwm,

                                   control_output.right_pwm);

                }

            }

        }

        else

        {

            DriveMotor_Stop();

            DriveMotionProfile_Reset(&motion_profile_state);

            ImuHeadingHold_Reset(&heading_hold_state);

            g_drive_command_age_ms = 0U;

            g_drive_left_pwm = 0;

            g_drive_right_pwm = 0;

            g_heading_hold_active = 0U;

            g_heading_reference_rad = 0.0f;

            g_heading_error_rad = 0.0f;

            g_heading_correction_rad_s = 0.0f;

        }

        if (!command_active ||

            (active_command.command_type != AMR_MOTION_COMMAND_WHEEL_SPEED))

        {

            g_motion_profile_status = DRIVE_MOTION_PROFILE_IDLE;

            g_motion_profile_fault = 0U;

        }

        if (g_heading_hold_active == 0U)

        {

            g_heading_desired_yaw_rate_rad_s = 0.0f;

            g_heading_yaw_rate_error_rad_s = 0.0f;

            g_heading_rate_ff_correction_rad_s = 0.0f;

            g_heading_rate_p_correction_rad_s = 0.0f;

            g_heading_rate_i_correction_rad_s = 0.0f;

            g_heading_controller_version =

                (IMU_HEADING_HOLD_V2_ENABLE != 0) ? 2U : 1U;

            g_heading_correction_saturated = 0U;

        }

        next_wake += 5U; /* Chu ky dieu khien 200 Hz */

        (void)osDelayUntil(next_wake);

    }

  /* USER CODE END StartMotorCtrlTask */

}

/* USER CODE BEGIN Header_StartCommTask */

/**

* @brief Function implementing the CommTask thread.

* @param argument: Not used

* @retval None

*/

/* USER CODE END Header_StartCommTask */

void StartCommTask(void *argument)

{

  /* USER CODE BEGIN StartCommTask */

    AppComm_Run();

    /* Task must not return */

    for (;;)

    {

        osDelay(1000);

    }

  /* USER CODE END StartCommTask */

}

/* USER CODE BEGIN Header_StartImuTask */

/**

* @brief Function implementing the ImuTask thread.

* @param argument: Not used

* @retval None

*/

/* USER CODE END Header_StartImuTask */

void StartImuTask(void *argument)

{

  /* USER CODE BEGIN StartImuTask */

    const uint32_t period_ms = 10U; /* 100 Hz */

    Bno055Device_t imu = {0};

    Bno055Sample_t sample = {0};

    ImuHeadingState_t heading_state = {0};

    ImuHeadingOutput_t heading_output = {0};

    uint32_t next_wake = osKernelGetTickCount();

    uint32_t consecutive_read_errors = 0U;

    bool initialized = false;

    /* De cac nguon 3.3 V va 5 V on dinh sau khi khoi dong. */

    osDelay(200U);

    for (;;)

    {

        if (!initialized)

        {

            g_imu_ready = 0U;

            g_imu_who_am_i = 0U;

            g_imu_raw_gyro_z_dps = 0.0f;

            g_imu_filtered_gyro_z_dps = 0.0f;

            g_imu_bno_fused_yaw_rad = 0.0f;

            g_imu_yaw_deg = 0.0f;

            g_imu_bno_fused_yaw_deg = 0.0f;

            g_imu_stationary_count = 0U;

            g_imu_rejected_orientation_count = 0U;

            (void)osEventFlagsClear(SystemEventsHandle,

                                    AMR_EVT_IMU_READY);

            if (!Bno055_Init(&imu, &hi2c1))

            {

                ++g_imu_error_count;

                osDelay(500U);

                continue;

            }

            ImuHeading_Init(&heading_state);

            heading_output = (ImuHeadingOutput_t){0};

            consecutive_read_errors = 0U;

            g_imu_who_am_i = imu.chip_id;

            next_wake = osKernelGetTickCount();

            initialized = true;

        }

        if (Bno055_ReadSample(&imu, &sample))

        {

            consecutive_read_errors = 0U;

            ++g_imu_sample_count;

            g_imu_accel_x_m_s2 = sample.accel_m_s2[0];

            g_imu_accel_y_m_s2 = sample.accel_m_s2[1];

            g_imu_accel_z_m_s2 = sample.accel_m_s2[2];

            g_imu_temperature_c = sample.temperature_c;

            /* Raw source-side diagnostics before ImuHeading filtering. */

            g_imu_raw_gyro_z_dps = sample.gyro_dps[2];

            g_imu_bno_fused_yaw_rad = sample.fused_yaw_rad;

            ImuHeading_Update(&heading_state,

                              &sample,

                              0.010f,

                              &heading_output);

            /* Manual relative-yaw zero.

             *

             * IMPORTANT:

             * - no STM32 reset

             * - no BNO055 reset

             * - no recalibration

             * - no ROS reset

             *

             * The request is consumed only when the estimator is READY and

             * this sample is valid. If not ready yet, leave request asserted

             * and expose status=2 so the operator can see it is waiting.

             */

            if (g_imu_zero_request != 0U)

            {

                if (ImuHeading_ZeroYaw(&heading_state,

                                       &sample,

                                       &heading_output))

                {

                    ++g_imu_zero_count;

                    g_imu_zero_status = 1U;

                    g_imu_zero_request = 0U;

                }

                else

                {

                    g_imu_zero_status = 2U;

                }

            }

            else if (g_imu_zero_status == 1U)

            {

                /* Keep SUCCESS visible until the next normal sample, then

                 * return to IDLE. */

                g_imu_zero_status = 0U;

            }

            g_imu_calibration_count =

                heading_output.calibration_count;

            g_imu_calibration_reset_count =

                heading_output.calibration_reset_count;

            g_imu_gyro_bias_z_dps =

                heading_output.gyro_bias_z_dps;

            g_imu_gyro_z_dps = heading_output.gyro_z_dps;

            g_imu_gyro_z_rad_s = heading_output.gyro_z_rad_s;

            g_imu_yaw_rad = heading_output.yaw_rad;

            /* Display only: convert radians -> degrees.

             * Does not feed back into control.

             */

            g_imu_yaw_deg = g_imu_yaw_rad * 57.29577951308232f;

            g_imu_bno_fused_yaw_deg =

                g_imu_bno_fused_yaw_rad * 57.29577951308232f;

            /* Internal anti-noise filter diagnostics for CubeIDE Live Expressions. */

            g_imu_filtered_gyro_z_dps = heading_state.filtered_gyro_z_dps;

            g_imu_stationary_count = heading_state.stationary_count;

            g_imu_rejected_orientation_count =

                heading_state.rejected_orientation_count;

            if (heading_output.ready)

            {

                g_imu_ready = 1U;

                (void)osEventFlagsSet(SystemEventsHandle,

                                      AMR_EVT_IMU_READY);

            }

            else

            {

                /* Fail closed: never leave a stale READY flag asserted if

                 * the source filter rejects orientation or re-calibrates.

                 */

                g_imu_ready = 0U;

                (void)osEventFlagsClear(SystemEventsHandle,

                                        AMR_EVT_IMU_READY);

            }

        }

        else

        {

            ++g_imu_error_count;

            ++consecutive_read_errors;

            if (consecutive_read_errors >= 10U)

            {

                initialized = false;

                g_imu_ready = 0U;

                (void)osEventFlagsClear(SystemEventsHandle,

                                        AMR_EVT_IMU_READY);

            }

        }

        next_wake += period_ms;

        (void)osDelayUntil(next_wake);

    }

  /* USER CODE END StartImuTask */

}

/* USER CODE BEGIN Header_StartTelemetryTask */

/**

* @brief Function implementing the TelemetryTask thread.

* @param argument: Not used

* @retval None

*/

/* USER CODE END Header_StartTelemetryTask */

void StartTelemetryTask(void *argument)

{

  /* USER CODE BEGIN StartTelemetryTask */

    const uint32_t period_ms = 20U; /* Chan doan 50 Hz, du nhe cho USB CDC. */

    uint32_t next_wake = osKernelGetTickCount();

    for (;;)

    {

        const float encoder_yaw_rate_rad_s =

            ((DRIVE_WHEEL_DIAMETER_M * 0.5f) *

             (g_wheel_right_speed_rad_s -

              g_wheel_left_speed_rad_s)) /

            TURN_CONTROLLER_WHEEL_SEPARATION_M;

        const uint32_t command_age_ms = g_drive_command_age_ms;

        const TelemetryData_t telemetry =

        {

            .timestamp_ms = osKernelGetTickCount(),

            .target_left_rad_s = g_drive_target_left_rad_s,

            .target_right_rad_s = g_drive_target_right_rad_s,

            .measured_left_rad_s = g_wheel_left_speed_rad_s,

            .measured_right_rad_s = g_wheel_right_speed_rad_s,

            .pwm_left = g_drive_left_pwm,

            .pwm_right = g_drive_right_pwm,

            .imu_yaw_rad = g_imu_yaw_rad,

            .gyro_z_rad_s = g_imu_gyro_z_rad_s,

            .heading_reference_rad = g_heading_reference_rad,

            .heading_error_rad = g_heading_error_rad,

            .heading_correction_rad_s =

                g_heading_correction_rad_s,

            .desired_yaw_rate_rad_s =

                g_heading_desired_yaw_rate_rad_s,

            .yaw_rate_error_rad_s =

                g_heading_yaw_rate_error_rad_s,

            .yaw_rate_ff_correction_rad_s =

                g_heading_rate_ff_correction_rad_s,

            .yaw_rate_p_correction_rad_s =

                g_heading_rate_p_correction_rad_s,

            .yaw_rate_i_correction_rad_s =

                g_heading_rate_i_correction_rad_s,

            .encoder_yaw_rate_rad_s = encoder_yaw_rate_rad_s,

            .yaw_rate_residual_rad_s =

                g_imu_gyro_z_rad_s - encoder_yaw_rate_rad_s,

            .accel_x_m_s2 = g_imu_accel_x_m_s2,

            .accel_y_m_s2 = g_imu_accel_y_m_s2,

            .accel_z_m_s2 = g_imu_accel_z_m_s2,

            .odom_x_m = g_odom_x_m,

            .odom_y_m = g_odom_y_m,

            .odom_yaw_rad = g_odom_yaw_rad,

            .linear_velocity_m_s = g_odom_linear_velocity_m_s,

            .angular_velocity_rad_s = g_odom_angular_velocity_rad_s,

            .sample_count = g_odom_sample_count,

            .command_refresh_count = g_drive_command_refresh_count,

            .command_age_ms =

                (command_age_ms > UINT16_MAX) ?

                UINT16_MAX : (uint16_t)command_age_ms,

            .command_active = g_drive_command_active,

            .heading_hold_active = g_heading_hold_active,

            .imu_ready = g_imu_ready,

            .odom_valid = g_odom_ready,

            .mode = AppComm_GetMode(),

            .heading_controller_version =

                g_heading_controller_version,

            .heading_correction_saturated =

                g_heading_correction_saturated,

            .motion_profile_status = g_motion_profile_status,

            .motion_profile_fault = g_motion_profile_fault

        };

        /* Khong block task; neu day thi bo snapshot cu nhat. */

        if (osMessageQueuePut(TelemetryQueueHandle,

                              &telemetry,

                              0U,

                              0U) != osOK)

        {

            TelemetryData_t discarded;

            (void)osMessageQueueGet(TelemetryQueueHandle,

                                    &discarded,

                                    NULL,

                                    0U);

            (void)osMessageQueuePut(TelemetryQueueHandle,

                                    &telemetry,

                                    0U,

                                    0U);

        }

        next_wake += period_ms;

        (void)osDelayUntil(next_wake);

    }

  /* USER CODE END StartTelemetryTask */

}

/* USER CODE BEGIN Header_StartDiagTask */

/**

* @brief Function implementing the DiagTask thread.

* @param argument: Not used

* @retval None

*/

/* USER CODE END Header_StartDiagTask */

void StartDiagTask(void *argument)

{

  /* USER CODE BEGIN StartDiagTask */

  /* Infinite loop */

  for(;;)

  {

     osDelay(100);

  }

  /* USER CODE END StartDiagTask */

}

/* Private application code --------------------------------------------------*/

/* USER CODE BEGIN Application */

/* USER CODE END Application */

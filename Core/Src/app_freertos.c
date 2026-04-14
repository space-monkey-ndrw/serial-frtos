/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
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
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "tim.h"
#include "stm32g4xx_nucleo.h"
#include "MadgwickAHRS.h"
#include "IMUinfo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
  float accel_x, accel_y, accel_z;
  float gyro_x, gyro_y, gyro_z;
  float qw, qx, qy, qz;
} ImuPacket_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MOTOR_MAX_PWM 99
#define SERIAL_RX_BUF_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t motor_rx_buf[SERIAL_RX_BUF_SIZE]; // Buffer for UART reception
extern UART_HandleTypeDef huart2;
extern __IO uint32_t BspButtonState;
extern I2C_HandleTypeDef hi2c1; // Declare the I2C handle defined in i2c.c
extern uint8_t imu_buf[12]; // 6 bytes for Gyro, 6 for Accel
extern uint8_t mag_buf[6];  // 6 bytes for Mag
float gx, gy, gz;
float ax, ay, az;
float mx, my, mz;
volatile int16_t raw_gx, raw_gy, raw_gz;
volatile int16_t raw_ax, raw_ay, raw_az;
volatile int16_t raw_mx, raw_my, raw_mz;
extern volatile float q0, q1, q2, q3; // Quaternion components from Madgwick filter
ImuPacket_t imu_packet;
// size is 40 Bytes + 1 (COBS overhead) + 1 (0x00 delimeter) = 42 Bytes total, well under our 64 Byte UART limit
uint8_t imu_packet_buf[42]; // Buffer to hold the serialized ImuPacket for UART transmission
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 1024 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for madgwickTask */
osThreadId_t madgwickTaskHandle;
const osThreadAttr_t madgwickTask_attributes = {
  .name = "madgwickTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 512 * 4
};
/* Definitions for motorCtrlTask */
osThreadId_t motorCtrlTaskHandle;
const osThreadAttr_t motorCtrlTask_attributes = {
  .name = "motorCtrlTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Motor_A_Fwd(int);
void Motor_A_Back(int);
void Motor_A_Stop(void);
void Motor_B_Fwd(int);
void Motor_B_Back(int);
void Motor_B_Stop(void);
void Motors_Fwd(int);
void Motors_Back(int);
void Motors_Stop(void);
void print_val_float(char* label, float val);
void print_val(char* label, int16_t val);
void parse_imu_data(uint8_t *imu_buf);
void parse_mag_data(uint8_t *mag_buf);
size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output);
size_t cobs_decode(const uint8_t *input, uint8_t *output, size_t length);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartMadgwickTask(void *argument);
void StartMotorCtrlTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of madgwickTask */
  madgwickTaskHandle = osThreadNew(StartMadgwickTask, NULL, &madgwickTask_attributes);

  /* creation of motorCtrlTask */
  motorCtrlTaskHandle = osThreadNew(StartMotorCtrlTask, NULL, &motorCtrlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  // 1. Wake LSM6DSOX Accel & Gyro
  // --- LSM6DSOX Initialization (104Hz) ---
  uint8_t lsm_accel_init[] = {0x10, 0x40}; // CTRL1_XL: 104Hz, +/- 2 g
  uint8_t lsm_gyro_init[]  = {0x11, 0x4C}; // CTRL2_G: 104Hz, 2000dps (Better for robots)
  uint8_t lsm_bdu[]        = {0x12, 0x44}; // CTRL3_C: BDU=1, IF_INC=1

  HAL_I2C_Master_Transmit(&hi2c1, LSM6DSOX_sADDR, lsm_accel_init, 2, 100);
  HAL_I2C_Master_Transmit(&hi2c1, LSM6DSOX_sADDR, lsm_gyro_init, 2, 100);
  HAL_I2C_Master_Transmit(&hi2c1, LSM6DSOX_sADDR, lsm_bdu, 2, 100);

  // --- LIS3MDL Initialization (10Hz Ultra-High) ---
  // CTRL_REG1: 0x70 sets OM=11 (Ultra-high X/Y) and DO=100 (10Hz)
  uint8_t mag_reg1[] = {0x20, 0x70}; 
  // CTRL_REG2: 0x00 sets FS=00 (+/- 4 Gauss)
  uint8_t mag_reg2[] = {0x21, 0x00}; 
  // CTRL_REG3: 0x00 sets MD=00 (Continuous conversion)
  uint8_t mag_reg3[] = {0x22, 0x00}; 
  // CTRL_REG4: 0x0C sets OMZ=11 (Ultra-high Z-axis)
  uint8_t mag_reg4[] = {0x23, 0x0C}; 
  HAL_I2C_Master_Transmit(&hi2c1, LIS3MDL_sADDR, mag_reg1, 2, 100);
  HAL_I2C_Master_Transmit(&hi2c1, LIS3MDL_sADDR, mag_reg2, 2, 100);
  HAL_I2C_Master_Transmit(&hi2c1, LIS3MDL_sADDR, mag_reg3, 2, 100);
  HAL_I2C_Master_Transmit(&hi2c1, LIS3MDL_sADDR, mag_reg4, 2, 100);

  osDelay(100); // Wait for sensors to stabilize

  HAL_StatusTypeDef status = HAL_TIM_Base_Start_IT(&htim3);
  if (status != HAL_OK) {
      __BKPT(0); // If it stops here, the timer failed to start
  }

  // Start PWM on TIM1 channels 1 and 2 to control the motors
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  // Drive STBY pin HIGH to enable the driver
  HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);

  /* Infinite loop */
  for(;;)
  {
    osDelay(100); // 10Hz loop rate is plenty for testing

    // -- Sample board code for User push-button in interrupt mode ----
    if (BspButtonState == BUTTON_PRESSED)
    {
      // Update button state
      BspButtonState = BUTTON_RELEASED;
      // -- Sample board code to toggle leds ----
      BSP_LED_Toggle(LED_GREEN);

      // ..... Perform your action .....

      // --- MOTOR A & B FORWARD  20% (200/1000) ---
      Motors_Fwd(125);
      osDelay(1000); // Run for 1000ms

      // --- STOP ---
      Motors_Stop();
      osDelay(1000); // Run for 1000ms

      // --- MOTOR A & B REVERSE 10% (100/1000) ---
      Motors_Back(50);
      osDelay(1000); // Run for 1000ms

      // --- STOP ---
      Motors_Stop();
      osDelay(1000); // Run for 1000ms

      // --- MOTOR A FORWARD & B REVERSE ---
      Motor_A_Fwd(50);
      Motor_B_Back(50);
      osDelay(1000); // Run for 1000ms

      // --- STOP ---
      Motors_Stop();
    }
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartMadgwickTask */
/**
* @brief Function implementing the madgwickTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMadgwickTask */
void StartMadgwickTask(void *argument)
{
  /* USER CODE BEGIN StartMadgwickTask */

  uint16_t test_counter = 0;
  volatile uint16_t dropped_packet_counter = 0;

  // 1. Initialize local variables
  uint32_t notifyValue = 0;
  float deltat = 1.0f / 104.0f; // Matching our 104Hz ODR
  /* Infinite loop */
  for(;;)
  {
    // Wait for notification and store the value in notifyValue
    xTaskNotifyWait(0, 0xFFFFFFFF, &notifyValue, portMAX_DELAY);

    // 3. Convert raw I2C bytes to floats and run the CORDIC-accelerated filter
    // (insert Madgwick calls and Cordic_InvSqrt here)
    // Assume 'imu_data' and 'mag_data' buffers are filled via DMA
    parse_imu_data(imu_buf);

    // 4. Optionally, only run the Mag update every 10th IMU read to save CPU
    if (notifyValue == DATA_READY_9DOF) {  
      // Perform MARG updates here (e.g., heading calculation)
      parse_mag_data(mag_buf);
      MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
    } else {
      // Perform ARG-only updates here (e.g., pitch/roll calculation)
      MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
    }
    
    test_counter++;

    // output for testing
    /*
    if (test_counter > 5) {
      test_counter = 0;
      print_val("raw_ax", raw_ax);
      print_val("raw_ay", raw_ay);
      print_val("raw_az", raw_az);
      print_val("raw_gx", raw_gx);
      print_val("raw_gy", raw_gy);
      print_val("raw_gz", raw_gz);
      print_val("raw_mx", raw_mx);
      print_val("raw_my", raw_my);
      print_val("raw_mz", raw_mz);
      HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 2, 10);
    }
    //end of output for testing
    */

    /*
    print_val_float("q0: ", q0);
    print_val_float("q1: ", q1);
    print_val_float("q2: ", q2);
    print_val_float("q3: ", q3);
    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 10);
    */

    // 5. Quaternion is now updated.  Prepare for Serial Bridge.
    //    pack 'q[0,3]' into ROS2 message and/or send over UART for debugging.
    imu_packet.accel_x = ax;
    imu_packet.accel_y = ay;
    imu_packet.accel_z = az;
    imu_packet.gyro_x = gx;
    imu_packet.gyro_y = gy;
    imu_packet.gyro_z = gz;
    imu_packet.qw = q0;
    imu_packet.qx = q1;
    imu_packet.qy = q2;
    imu_packet.qz = q3;

    size_t encoded_length = cobs_encode((uint8_t*)&imu_packet, sizeof(ImuPacket_t), imu_packet_buf);
    // ONLY transmit if the previous DMA transfer is finished
    if (huart2.gState == HAL_UART_STATE_READY) {
      HAL_UART_Transmit_DMA(&huart2, imu_packet_buf, encoded_length);
    } else {
      dropped_packet_counter++;
      // Optional: Log a "Dropped Packet" count for debugging
    }
  }
  /* USER CODE END StartMadgwickTask */
}

/* USER CODE BEGIN Header_StartMotorCtrlTask */
/**
* @brief Function implementing the motorCtrlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorCtrlTask */
void StartMotorCtrlTask(void *argument)
{
  /* USER CODE BEGIN StartMotorCtrlTask */

  float left_cmd = 0.0f;
  float right_cmd = 0.0f;
  uint8_t decoded_buf[16]; // buffer for decoded floats
  uint32_t received_size = 0;

  /* Infinite loop */
  for(;;)
  {
    // wait for notification and capture the size passed from vTaskNotifyGiveFromISR
    // use 500ms timeout as safety watchdog
    received_size = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));

    if (received_size == 10) { // We expect 10 bytes: 1 COBS code byte + 8 data bytes + 1 delimiter
      // COBS decode - need to implement
      cobs_decode(motor_rx_buf, decoded_buf, received_size-1); // Exclude the delimiter from decoding
      // Unpack: convert bytes to floats
      memcpy(&left_cmd, &decoded_buf[0], 4);
      memcpy(&right_cmd, &decoded_buf[4], 4);

      if (left_cmd > 0) {
        Motor_A_Fwd((int)left_cmd);
      } else if (left_cmd < 0) {
        Motor_A_Back((int)(-left_cmd)); // invert for reverse
      } else {
        Motor_A_Stop();
      }

      if (right_cmd > 0) {
        Motor_B_Fwd((int)right_cmd);
      } else if (right_cmd < 0) {
        Motor_B_Back((int)(-right_cmd)); // invert for reverse
      } else {
        Motor_B_Stop();
      }

      // restart DMA for next packet
      HAL_UARTEx_ReceiveToIdle_DMA(&huart2, motor_rx_buf, sizeof(motor_rx_buf));

    } else {
      // Optional: Implement a timeout behavior, e.g., stop motors if no command received for 500ms
      Motors_Stop();
    }
  }
  /* USER CODE END StartMotorCtrlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
 * @brief Consistent Overhead Byte Stuffing (COBS) Encoding
 * @param input: Pointer to the raw data struct
 * @param length: Size of the raw data (sizeof(ImuPacket_t))
 * @param output: Pointer to the buffer where encoded data + 0x00 will be stored
 * @return size_t: The total length of the encoded packet including the delimiter
 */
size_t cobs_encode(const uint8_t *input, size_t length, uint8_t *output) {
    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    while (read_index < length) {
        if (input[read_index] == 0) {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            output[write_index++] = input[read_index++];
            code++;
            if (code == 0xFF) {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    output[code_index] = code;
    output[write_index] = 0x00; // Add the framing delimiter
    return write_index + 1;
}

/**
 * @brief Consistent Overhead Byte Stuffing (COBS) decoding
 * @param input: Pointer to the raw data struct
 * @param output: Pointer to the buffer where encoded data + 0x00 will be stored
 * @param length: Size of the raw data (sizeof(ImuPacket_t))
 * @return size_t: The total length of the encoded packet including the delimiter
 */
size_t cobs_decode(const uint8_t *input, uint8_t *output, size_t length) {
    const uint8_t *start = output;
    while (length > 0)
    {
        uint8_t code = *input++;
        if (code == 0) break; // Should not happen in a valid COBS packet before the end
        length--;
        
        if (code > length + 1) return 0; // Error: packet is too short for the code provided

        for (uint8_t i = 1; i < code; i++)
        {
            *output++ = *input++;
            length--;
        }

        if (code < 0xFF && length > 0)
        {
            *output++ = 0;
        }
    }
    return (size_t)(output - start);
}

void print_val(char* label, int16_t val) {
    char msg[64];
    char sign = (val >= 0) ? ' ' : '-';
    if (val < 0) val = -val; // Make val positive for printing
    // Standard sprintf (no floats) is much smaller than float-enabled printf
    int len = sprintf(msg, "%s: %c%ld ", label, sign, val);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 10);
}

void print_val_float(char* label, float val) {
    char msg[32];
    char sign = (val >= 0) ? ' ' : '-';
    if (val < 0) val = -val; // Make val positive for printing
    int32_t whole = (int32_t)val;
    int32_t frac = (int32_t)((val - (float)whole) * 100000.0f + 0.5f); // 5 decimal places, add 0.5f for rounding up

    // Standard sprintf (no floats) is much smaller than float-enabled printf
    int len = sprintf(msg, "%s: %c%ld.%05ld  ", label, sign, whole, frac);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 10);
}

void parse_imu_data(uint8_t imu_buf[12]) {
  // measured offsets:
  
  float gx_offset = -0.46944f;
  float gy_offset = 0.52472f;
  float gz_offset = -0.27444f;
  float ax_offset = -0.01726f;
  float ay_offset = -0.04193f;
  float az_offset = -0.00174f;
  

  // DS12814 Rev 4: Gyro (0x22-0x27) comes BEFORE Accel (0x28-0x2D)
  raw_gx = (int16_t)((imu_buf[1] << 8) | imu_buf[0]);
  raw_gy = (int16_t)((imu_buf[3] << 8) | imu_buf[2]);
  raw_gz = (int16_t)((imu_buf[5] << 8) | imu_buf[4]);
  raw_ax = (int16_t)((imu_buf[7] << 8) | imu_buf[6]);
  raw_ay = (int16_t)((imu_buf[9] << 8) | imu_buf[8]);
  raw_az = (int16_t)((imu_buf[11] << 8) | imu_buf[10]);

  // 3. Conversion to physical units
  // +/- 2000dps: 70 mdps/LSB = 0.070 dps/LSB
  gx = (float)raw_gx * 0.070f - gx_offset;
  gy = (float)raw_gy * 0.070f - gy_offset;
  gz = (float)raw_gz * 0.070f - gz_offset;
  // convert to rad/s for Madgwick filter
  gx *= 0.0174533f; // π/180
  gy *= 0.0174533f; // π/180
  gz *= 0.0174533f; // π/180

  // Convert to g (at ±2g range, sensitivity is 0.061 mg/LSB)
  // +/- 2g: 0.061 mg/LSB = 0.000061 g/LSB
  ax = (float)raw_ax * 0.000061f - ax_offset;
  ay = (float)raw_ay * 0.000061f - ay_offset;
  az = (float)raw_az * 0.000061f - az_offset;
}

void parse_mag_data(uint8_t mag_buf[6]) {
  // measured offsets:
  float mx_offset = -0.03077f;
  float my_offset = -0.12533f;
  float mz_offset = -0.044f;

  raw_mx = (int16_t)((mag_buf[1] << 8) | mag_buf[0]);
  raw_my = (int16_t)((mag_buf[3] << 8) | mag_buf[2]);
  raw_mz = (int16_t)((mag_buf[5] << 8) | mag_buf[4]);
  // Mag (Registers 0x28 to 0x2D)
  // Convert to Gauss (at ±4 Gauss range, sensitivity is 6842 LSB/Gauss)
  mx = (float)raw_mx / 6842.0f - mx_offset;
  my = (float)raw_my / 6842.0f - my_offset;
  mz = (float)raw_mz / 6842.0f - mz_offset;
}

void Motor_A_Fwd(int speed)
{
	if ((speed > MOTOR_MAX_PWM) | (speed < 0))
	{
		return;
	}
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET); // AIN1 LOW
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);   // AIN2 HIGH
  htim1.Instance->CCR1 = speed; // Direct register access for faster updates
}

void Motor_A_Back(int speed)
{
	if ((speed > MOTOR_MAX_PWM) | (speed < 0))
	{
		return;
	}
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET); // AIN1 HIGH
	HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);   // AIN2 LOW
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);    // % speed (speed/MOTOR_MAX_PWM)
}

void Motor_A_Stop(void)
{
	HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
  
  htim1.Instance->CCR1 = 0; // Ensure PWM duty cycle is also set to 0
}

void Motor_B_Fwd(int speed)
{
	if ((speed > MOTOR_MAX_PWM) | (speed < 0))
	{
		return;
	}
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET); // BIN1 LOW
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);   // BIN2 HIGH
	htim1.Instance->CCR2 = speed; // Direct register access for faster updates
}

void Motor_B_Back(int speed)
{
	if ((speed > MOTOR_MAX_PWM) | (speed < 0))
	{
		return;
	}
	HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET); // BIN1 HIGH
	HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);   // BIN2 LOW
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, speed);    // % speed (speed/MOTOR_MAX_PWM)
}

void Motor_B_Stop(void)
{
  HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
  
  htim1.Instance->CCR2 = 0; // Ensure PWM duty cycle is also set to 0
}

void Motors_Fwd(int speed)
{
	Motor_A_Fwd(speed);
	Motor_B_Fwd(speed);
}

void Motors_Back(int speed)
{
	Motor_A_Back(speed);
	Motor_B_Back(speed);
}

void Motors_Stop(void)
{
	Motor_A_Stop();
	Motor_B_Stop();
}
/* USER CODE END Application */


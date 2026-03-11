/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "cordic.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IMUinfo.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

__IO uint32_t BspButtonState = BUTTON_RELEASED;

/* USER CODE BEGIN PV */
uint8_t imu_buf[12]; // 6 bytes for Gyro, 6 for Accel
uint8_t mag_buf[6];  // 6 bytes for Mag
float gx, gy, gz;
float ax, ay, az;
float mx, my, mz;
volatile uint8_t mag_pending = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_CORDIC_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // testing code below
  //uncommend the line below to restore non-testing code
  // HAL_TIM_Base_Start_IT(&htim3); // Start timer for periodic sensor reading (IMU) task
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* USER CODE BEGIN BSP */

  /* -- Sample board code to switch on leds ---- */
  BSP_LED_On(LED_GREEN);

  /* USER CODE END BSP */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* -- Sample board code for User push-button in interrupt mode ---- */
    if (BspButtonState == BUTTON_PRESSED)
    {
      /* Update button state */
      BspButtonState = BUTTON_RELEASED;
      /* -- Sample board code to toggle leds ---- */
      BSP_LED_Toggle(LED_GREEN);

      /* ..... Perform your action ..... */
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void* __wrap_malloc(size_t size) { return pvPortMalloc(size); }
void __wrap_free(void* ptr) { vPortFree(ptr); }

/* --- IMU DATA CALLBACK --- */
extern osThreadId_t madgwickTaskHandle; // From app_freertos.c

/**
  * @brief Override the __weak HAL function to wake our Madgwick Task
  */
// Note: Changed from MemRxCplt to MasterRxCplt to match Master_Receive_DMA
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  static uint8_t mag_counter = 0;

  if (hi2c->Instance == I2C1)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    //  --- CASE A: IMU Read Finished (12 bytes) ---
    if (hi2c->XferSize == 12) {
        mag_counter++;

        if (mag_counter < 10) {
          xTaskNotifyFromISR(madgwickTaskHandle, DATA_READY_6DOF, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
        } else {
          mag_counter = 0;
          mag_pending = 1; // Lock the state: we are now doing a Mag read

          // start mag read
          // LIS3MDL_reg (reg addr 0x28) | 0x80 (auto-increment bit set) to read all 6 bytes in one go
          static uint8_t mag_addr_auto = 0x28 | 0x80;
          if (HAL_I2C_Master_Transmit(&hi2c1, LIS3MDL_sADDR, &mag_addr_auto, 1, 10) == HAL_OK) {
              // 2. Start the DMA Receive (Non-blocking)
              // When THIS finishes, this Callback will fire again with hi2c->XferSize == 6
              HAL_I2C_Master_Receive_DMA(&hi2c1, LIS3MDL_sADDR, mag_buf, 6);
          } else {
              mag_pending = 0; // If Transmit fails, reset the lock so we don't hang
              // Still signal 6-DOF update so we don't lose IMU data
              xTaskNotifyFromISR(madgwickTaskHandle, DATA_READY_6DOF, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
          }
        }
    } 
    //  --- CASE B: Magnetometer Read Finished (6 bytes) ---
    else if (hi2c->XferSize == 6) {
      mag_pending = 0; // Unlock the state: Mag read is done
      // Signal 9-DOF and wake task
      xTaskNotifyFromISR(madgwickTaskHandle, DATA_READY_9DOF, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    }
    
    /* Force a context switch if MadgwickTask is higher priority 
      than the task the CPU was running when the I2C finished. */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

// This triggers automatically when the Address Write finishes
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        // Start reading 12 bytes: Accel (6) + Gyro (6)
        HAL_I2C_Master_Receive_DMA(hi2c, LSM6DSOX_sADDR, imu_buf, 12);
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  // Add your IMU Trigger logic here
  if (htim->Instance == TIM3)
  {
    // Only trigger if a Magnetometer read isn't currently hogging the bus
    if (!mag_pending) {
      // Start I2C DMA Read at 0x22 (OUTX_L_A) to get Accel (6 bytes) + Gyro (6 bytes) 
      // Step 1: Set the pointer with a CLEAN STOP (No Repeated Start)
      // We use a small timeout (10ms) because this is only 1 byte.
      static uint8_t reg_addr = 0x22;
      if (HAL_I2C_Master_Transmit(&hi2c1, LSM6DSOX_sADDR, &reg_addr, 1, 10) == HAL_OK) {
        // Step 2: Now pull the 12 bytes using DMA
        // This is non-blocking. The CPU exits this interrupt immediately.
        if (hi2c1.State != HAL_I2C_STATE_READY) {
          // If you hit this breakpoint, the blocking Transmit didn't reset the state machine
          __BKPT(0); 
        }
        HAL_StatusTypeDef statusLSMread = HAL_I2C_Master_Receive_DMA(&hi2c1, LSM6DSOX_sADDR, imu_buf, 12);
        if (statusLSMread != HAL_OK) {
            __BKPT(0); // If it stops here, the DMA failed to start
        }
      }
    }
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pressed button
  * @retval None
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  if (Button == BUTTON_USER)
  {
    BspButtonState = BUTTON_PRESSED;
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

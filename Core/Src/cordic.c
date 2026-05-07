/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    cordic.c
  * @brief   This file provides code for the configuration
  *          of the CORDIC instances.
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
#include "cordic.h"

/* USER CODE BEGIN 0 */
// Constants for RM0440 n=0 constraints in Q1.31
#define CORDIC_MIN_Q31 0x0374BC6A  // 0.027 * 2^31
#define CORDIC_MAX_Q31 0x60000000  // 0.75  * 2^31
/* USER CODE END 0 */

CORDIC_HandleTypeDef hcordic;

/* CORDIC init function */
void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */
  CORDIC_ConfigTypeDef sCordicConfig_Sqrt;

  sCordicConfig_Sqrt.Function         = CORDIC_FUNCTION_SQUAREROOT; // Calculate SQRT
  sCordicConfig_Sqrt.Precision        = CORDIC_PRECISION_6CYCLES;   // Fast for Madgwick
  sCordicConfig_Sqrt.Scale            = CORDIC_SCALE_0;             // No internal shift
  sCordicConfig_Sqrt.NbWrite          = CORDIC_NBWRITE_1;           // 1 input (x)
  sCordicConfig_Sqrt.NbRead           = CORDIC_NBREAD_1;            // 1 output (sqrt(x))
  sCordicConfig_Sqrt.InSize           = CORDIC_INSIZE_32BITS;       // Q1.31
  sCordicConfig_Sqrt.OutSize          = CORDIC_OUTSIZE_32BITS;      // Q1.31

  if (HAL_CORDIC_Configure(&hcordic, &sCordicConfig_Sqrt) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END CORDIC_Init 2 */

}

void HAL_CORDIC_MspInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspInit 0 */

  /* USER CODE END CORDIC_MspInit 0 */
    /* CORDIC clock enable */
    __HAL_RCC_CORDIC_CLK_ENABLE();
  /* USER CODE BEGIN CORDIC_MspInit 1 */

  /* USER CODE END CORDIC_MspInit 1 */
  }
}

void HAL_CORDIC_MspDeInit(CORDIC_HandleTypeDef* cordicHandle)
{

  if(cordicHandle->Instance==CORDIC)
  {
  /* USER CODE BEGIN CORDIC_MspDeInit 0 */

  /* USER CODE END CORDIC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CORDIC_CLK_DISABLE();
  /* USER CODE BEGIN CORDIC_MspDeInit 1 */

  /* USER CODE END CORDIC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/**
  * @brief  Accelerated Inverse Square Root using G4 CORDIC Hardware.
  * @param  x: The value to calculate 1/sqrt(x) for.
  * @retval The inverse square root result as a float.
  */
float Cordic_InvSqrt(volatile float x) {
    if (x <= 0.0f) return 0.0f;

    // 1. Scale input to fit in Q1.31 [0, 1] range.
    // We scale by 0.25 to allow inputs up to 4.0 (covers normalized vectors).
    volatile float scaled_x = x * 0.25f; 
    
    // 2. Convert Float to Q1.31 Fixed Point
    int32_t q31_input = (int32_t)(scaled_x * 2147483648.0f);  // 2^31 for Q1.31
    int32_t q31_output = 0;

    // 3. Write to CORDIC and Read result (Blocking Mode)
    // This takes ~6-12 clock cycles (approx 35-70ns @ 170MHz)
    if (HAL_CORDIC_Calculate(&hcordic, &q31_input, &q31_output, 1, 10) != HAL_OK) {
        return 0.0f; // Handle error if needed
    }

    // 4. Convert back to Float and Correct the Scaling
    // sqrt(x/4) = sqrt(x)/2. Multiplying by 2.0f gets us back to sqrt(x).
    float result_sqrt = ((float)q31_output / 2147483648.0f) * 2.0f;

    // 5. Final Inverse
    return 1.0f / result_sqrt;
}

float Cordic_Sqrt(float input_f) {
    if (input_f <= 0.0f) return 0.0f;

    // 1. Convert float to a 64-bit bit-pattern (Q31 representation)
    // We use 64-bit to prevent overflow when input_f > 1.0
    int64_t bits = (int64_t)(input_f * 2147483648.0);
    int32_t k = 0; // This tracks the output shift

    // 2. NORMALIZE into [0.027, 0.75] using EVEN bit-shifts
    if (bits > CORDIC_MAX_Q31) {
        // Too big: Shift RIGHT by 2 (Input / 4), track k (Result * 2)
        while (bits > CORDIC_MAX_Q31) {
            bits >>= 2;
            k++; 
        }
    } else if (bits < CORDIC_MIN_Q31) {
        // Too small: Shift LEFT by 2 (Input * 4), track k (Result / 2)
        while (bits < CORDIC_MIN_Q31) {
            bits <<= 2;
            k--;
        }
    }

    // 3. CORDIC OPERATION (n=0)
    CORDIC->WDATA = (int32_t)bits;
    // Wait/Read Result
    int32_t result_q31 = CORDIC->RDATA;

    // 4. BIT-SHIFT CORRECTION
    // The relationship is: sqrt(x * 2^(2k)) = sqrt(x) * 2^k
    float res_f = (float)result_q31 / 2147483648.0f;

    // If we shifted input right (k > 0), result is too small; multiply by 2^k
    // If we shifted input left (k < 0), result is too big; divide by 2^k
    if (k > 0) return res_f * (float)(1 << k);
    if (k < 0) return res_f / (float)(1 << (-k));
    
    return res_f;
}
/* USER CODE END 1 */

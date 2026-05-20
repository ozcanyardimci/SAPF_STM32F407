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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* APF algorithm modules:
 * apf_hystctrl.h → inner loop (hysteresis controller)
 * apf_refgen.h   → outer loop (reference current generator) */
#include "apf_hystctrl.h"
#include "apf_refgen.h"
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

/* USER CODE BEGIN PV */
/* DMA transfer buffer — 4 ADC channels filled automatically
 * by DMA2 Stream0 at 20kHz, triggered by TIM1 CH2 (CCR2=4199).
 * Index map:
 *   [0] PA1 CH1  → I_APF  (ACS712-05B ±5A, bipolar)
 *   [1] PA2 CH2  → V_dc   (voltage divider 0-100V, unipolar)
 *   [2] PA3 CH3  → V_S    (voltage divider ±70V, bipolar)
 *   [3] PC1 CH11 → I_L1   (ACS712-05B ±5A, bipolar)
 * volatile: prevents compiler optimizing away ISR reads. */
volatile uint16_t adc_buf[4];

/* Debug variables — monitored via STM32CubeMonitor while
 * firmware runs. volatile ensures CubeMonitor always reads
 * the value written by the last ISR call, not a cached copy. */
volatile float debug_I_APF_ref = 0.0f;  /* A  reference current  */
volatile float debug_V_S       = 0.0f;  /* V  grid voltage        */
volatile float debug_I_L1      = 0.0f;  /* A  load current        */
volatile float debug_V_dc      = 0.0f;  /* V  DC link voltage     */
volatile float debug_P_avg     = 0.0f;  /* W  average active power*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* Step 1: Initialize algorithm state before hardware starts.
   * Both modules reset persistent variables and set TIM1->CCR1
   * to CCR1_GATE_OFF (20 counts = 119ns) — safe gate-off state.
   * Must be first: CCR1 must be set before timer starts.       */
  APF_RefGen_Init();
  APF_HystCtrl_Init();

  /* Step 2: Arm ADC DMA before the timer fires first trigger.
   * DMA must be ready to accept transfers from the first CC2 event.
   * Cast to uint32_t* required by HAL API signature.
   * DMA transfer width remains Half Word as configured in CubeMX. */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, 4);

  /* Step 3: Start PWM outputs before enabling the update interrupt.
   * Gate signals must be active before the ISR can write to CCR1.
   * CH1/CH1N drive the H-bridge (PA8 and PB13) with 500ns deadtime.
   * CH2 provides the fixed ADC trigger at CCR2=4199 (mid-period).  */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

  /* Step 4: Enable TIM1 update interrupt last.
   * __HAL_TIM_ENABLE_IT is used instead of HAL_TIM_Base_Start_IT
   * because HAL_TIM_PWM_Start already set htim1.State = BUSY.
   * HAL_TIM_Base_Start_IT checks State == READY and silently
   * returns HAL_ERROR without enabling the interrupt.
   * This macro writes directly to TIM1 DIER register,
   * bypassing the HAL state machine. Safe and correct.          */
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * HAL_TIM_PeriodElapsedCallback
 * ─────────────────────────────
 * TIM1 update interrupt service routine — executes every 50µs (20kHz).
 *
 * This is the main real-time control loop of the SAPF firmware.
 * It overrides the weak implementation in the HAL library.
 *
 * Execution sequence every 50µs:
 *   1. Scale raw ADC values from DMA buffer to physical units
 *   2. Run APF_RefGen outer loop → compute I_APF_ref
 *   3. Run APF_HystCtrl inner loop → drive H-bridge gate
 *   4. Update debug variables for CubeMonitor visibility
 *
 * Timing note:
 *   ADC trigger fires at CCR2=4199 (25µs into each 50µs period).
 *   DMA completes the 4-channel scan in 18.3µs.
 *   By the time this callback runs (at 50µs), adc_buf holds
 *   fresh values from the current cycle with 6.7µs margin.
 *
 * All arithmetic uses float — Cortex-M4F FPU handles single-
 * precision in one clock cycle at 168MHz.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* Guard: TIM1 update interrupt shares IRQ with TIM10.
     * Return immediately if the event is not from TIM1.  */
    if (htim->Instance != TIM1)
    {
        return;
    }

    /* ── Step 1: Scale raw ADC values to physical units ──────────
     *
     * Bipolar scaling (I_APF, V_S, I_L1):
     *   physical = ((raw - 2048.0f) / 2047.5f) × FULL_SCALE
     *   ADC mid-scale 2048 → 0 (zero current or zero voltage)
     *   ADC full-scale 4095 → +FULL_SCALE
     *   ADC zero       0    → -FULL_SCALE
     *
     * Unipolar scaling (V_dc):
     *   physical = (raw / 4095.0f) × FULL_SCALE
     *   ADC zero 0 → 0V, ADC full-scale 4095 → FULL_SCALE        */

    float I_APF = ((float)adc_buf[0] - 2048.0f)
                  / 2047.5f * I_APF_FULL_SCALE;  /* ±5A  */

    float V_dc  = ((float)adc_buf[1] / 4095.0f)
                  * V_DC_FULL_SCALE;              /* 0-100V */

    float V_S   = ((float)adc_buf[2] - 2048.0f)
                  / 2047.5f * V_S_FULL_SCALE;    /* ±70V */

    float I_L1  = ((float)adc_buf[3] - 2048.0f)
                  / 2047.5f * I_L1_FULL_SCALE;   /* ±5A  */

    /* ── Step 2: Compute reference compensation current ──────────
     * Seven-step instantaneous power theory algorithm.
     * Returns I_APF_ref in amps — the current the APF must inject
     * to cancel harmonic pollution from the nonlinear load.       */
    float I_APF_ref = APF_RefGen_Update(V_S, I_L1, V_dc);

    /* ── Step 3: Drive H-bridge gate ─────────────────────────────
     * Hysteresis controller compares I_APF_ref to actual I_APF.
     * Writes result directly to TIM1->CCR1.
     * gate=1 → CCR1=ARR  → Q1+Q4 ON (positive injection)
     * gate=0 → CCR1=20   → Q2+Q3 ON (negative injection)         */
    APF_HystCtrl_Update(I_APF_ref, I_APF);

    /* ── Step 4: Update debug variables ──────────────────────────
     * Updated after algorithm runs — CubeMonitor always sees a
     * consistent snapshot from the same ISR execution.            */
    debug_I_APF_ref = I_APF_ref;
    debug_V_S       = V_S;
    debug_I_L1      = I_L1;
    debug_V_dc      = V_dc;
    debug_P_avg     = APF_RefGen_GetPavg();
}
/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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

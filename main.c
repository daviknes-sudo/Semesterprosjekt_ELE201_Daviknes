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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> 
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_CSV 1             // 1 = send CSV lines "n,pulse_us,adc" for logging, 0 = send readable text
#define IR_SAMPLE_GAP_MS 3   // Pause between the 16 ADC samples: 16 x 3 ms = 48 ms, longer than one Sharp update cycle (about 38 ms)
#define IR_MIN_MV 380                              // lowest trusted IR voltage in mV (about 85 cm)
#define IR_MAX_MV 2600                             // highest trusted IR voltage in mV (about 10 cm)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
volatile uint32_t ic_rise = 0;     // Timer value when the echo pulse started (rising edge)
volatile uint32_t pulse_us = 0;    // Measured echo pulse width in microseconds
volatile uint8_t  new_pulse = 0;   // Flag: 1 = a fresh measurement is ready to be printed
volatile uint8_t  wait_fall = 0;   // State: 0 = waiting for rising edge, 1 = waiting for falling edge
uint32_t sample_n = 0;       // Counts measurements so every CSV line gets a running number
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void delay_us(uint32_t us)                                   // Busy-wait for a number of microseconds (uses TIM4, which counts 1 tick = 1 us)
{
  uint32_t start = __HAL_TIM_GET_COUNTER(&htim4);                   // Remember the timer value when we start waiting
  while (((__HAL_TIM_GET_COUNTER(&htim4) + 60000 - start) % 60000) < us) {}   // Wait until "us" ticks have passed (modulo handles wrap-around at 60000)
}

static void lcd_write_nibble(uint8_t nib)                           // Put 4 bits on D4..D7 and pulse E so the LCD reads them
{
  HAL_GPIO_WritePin(LCD_D4_GPIO_Port, LCD_D4_Pin, (nib >> 0) & 1);  // Bit 0 goes to D4
  HAL_GPIO_WritePin(LCD_D5_GPIO_Port, LCD_D5_Pin, (nib >> 1) & 1);  // Bit 1 goes to D5
  HAL_GPIO_WritePin(LCD_D6_GPIO_Port, LCD_D6_Pin, (nib >> 2) & 1);  // Bit 2 goes to D6
  HAL_GPIO_WritePin(LCD_D7_GPIO_Port, LCD_D7_Pin, (nib >> 3) & 1);  // Bit 3 goes to D7
  HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);      // E high: ring the doorbell
  delay_us(2);                                                      // Keep E high for at least 450 ns (we wait ~2 us to be safe)
  HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);    // E low: the LCD reads the 4 bits on this falling edge
  delay_us(50);                                                     // Give the LCD time to process (most commands need about 37 us)
}

static void lcd_send(uint8_t value, uint8_t rs)                     // Send one full byte as two nibbles; rs = 0 for command, 1 for character
{
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, rs);              // Set RS: 0 = command, 1 = character data
  lcd_write_nibble(value >> 4);                                     // Send the upper 4 bits first
  lcd_write_nibble(value & 0x0F);                                   // Then send the lower 4 bits
}

static void lcd_cmd(uint8_t cmd)  { lcd_send(cmd, 0); }             // Helper: send a command byte (RS = 0)
static void lcd_data(uint8_t ch)  { lcd_send(ch, 1); }              // Helper: send a character byte (RS = 1)

static void lcd_init(void)                                          // Wake up the LCD and put it in 4-bit mode (standard HD44780 sequence)
{
  HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);  // Start with RS low
  HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);    // Start with E low
  HAL_Delay(50);                                                    // Wait more than 40 ms after power-up so the LCD is ready
  lcd_write_nibble(0x03);                                           // Wake-up step 1: "8-bit mode" command, sent three times on purpose
  HAL_Delay(5);                                                     // Wait more than 4.1 ms
  lcd_write_nibble(0x03);                                           // Wake-up step 2
  delay_us(150);                                                    // Wait more than 100 us
  lcd_write_nibble(0x03);                                           // Wake-up step 3
  delay_us(150);                                                    // Short wait
  lcd_write_nibble(0x02);                                           // Now switch the LCD to 4-bit mode
  lcd_cmd(0x28);                                                    // Function set: 4-bit bus, 2 lines, 5x8 dot font
  lcd_cmd(0x0C);                                                    // Display on, cursor off, blinking off
  lcd_cmd(0x06);                                                    // Entry mode: cursor moves right after each character
  lcd_cmd(0x01);                                                    // Clear the display
  HAL_Delay(2);                                                     // Clear takes about 1.5 ms, so wait 2 ms
}

static void lcd_set_cursor(uint8_t row, uint8_t col)                // Move the cursor: row 0 = top line, row 1 = bottom line
{
  lcd_cmd(0x80 | ((row ? 0x40 : 0x00) + col));                      // 0x80 = "set cursor" command; bottom line starts at address 0x40
}

static void lcd_print(const char *s)                                // Print a text string starting at the current cursor position
{
  while (*s) lcd_data((uint8_t)*s++);                               // Send each character one by one until the string ends
}
static void buzzer_set(uint32_t freq_hz)                                // Play a tone at the given frequency in Hz (0 = silence)
{
  if (freq_hz == 0)                                                     // A frequency of zero means "be quiet"
  {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);                    // Duty 0 %: the output stays low, so no sound
    return;                                                             // Nothing more to do
  }
  uint32_t arr = 1000000 / freq_hz - 1;                                 // Timer ticks are 1 us, so the period in ticks = 1,000,000 / frequency (minus 1 for the register)
  __HAL_TIM_SET_AUTORELOAD(&htim1, arr);                                // Set the new period (this sets the pitch)
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (arr + 1) / 2);          // Set duty to 50 % (half the period high), which is the loudest
}

static void alert_update(uint32_t cm)                                   // Update LEDs and buzzer based on the distance in whole centimetres
{
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,                         // Green LED: on when the object is far away
                    (cm > 30) ? GPIO_PIN_SET : GPIO_PIN_RESET);         // Far = more than 30 cm
  HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin,                         // Yellow LED: on at medium distance
                    (cm > 15 && cm <= 30) ? GPIO_PIN_SET : GPIO_PIN_RESET);   // Medium = 16 to 30 cm
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin,                         // Red LED: on when the object is close
                    (cm <= 15) ? GPIO_PIN_SET : GPIO_PIN_RESET);        // Close = 15 cm or less

  if (cm >= 50)                                                         // Object is far away: no warning sound
  {
    buzzer_set(0);                                                      // Silence
  }
  else                                                                  // Object is within 50 cm: play a tone
  {
    uint32_t c = (cm < 5) ? 5 : cm;                                     // Clamp very small distances to 5 cm so the frequency stays in range
    buzzer_set(4500 - 30 * c);                                          // Frequency: 4350 Hz at 5 cm falling to 3000 Hz at 50 cm (closer = higher pitch, around the piezo's 4 kHz resonance)
  }
}
uint32_t ir_distance_mm(uint32_t mv)               // Convert the Sharp voltage in mV to a distance in mm
{
  const uint32_t a_k = 261800;                     // a = 26.18 cm*V, scaled so the result comes out in mm
  const uint32_t b_mv = 74;                        // offset b = 0.074 V, in mV
  if (mv < IR_MIN_MV || mv > IR_MAX_MV) return 0;  // outside the trusted range: report 0 (out of range)
  return a_k / (mv - b_mv);                        // d_mm = a_k / (mV - b)
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
char buf[80];                                                 // A box that can hold 80 characters of text (used by snprintf in the loop)

int n = snprintf(buf, sizeof buf,                             // Build the clock report text
                 "SYSCLK=%lu PCLK1=%lu PCLK2=%lu\r\n",        // Template: three clock speeds and a new line
                 (unsigned long)HAL_RCC_GetSysClockFreq(),    // System clock in Hz (should be 108000000)
                 (unsigned long)HAL_RCC_GetPCLK1Freq(),       // APB1 clock in Hz (should be 54000000)
                 (unsigned long)HAL_RCC_GetPCLK2Freq());      // APB2 clock in Hz (should be 108000000)
HAL_UART_Transmit(&huart3, (uint8_t*)buf, n, 100);            // Send the clock report to PuTTY once at startup

#if LOG_CSV                                                                     // Only compiled when CSV logging is on
const char csv_header[] = "n,pulse_us,adc\r\n";                                 // Column names: the first line of the log
HAL_UART_Transmit(&huart3, (uint8_t*)csv_header, sizeof csv_header - 1, 100);   // Send the header (minus the terminating zero)
#endif                                                                          // End of the CSV-only block
HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_3);                   // Start input capture on TIM4 CH3 with interrupts (listens to ECHO)
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);                     // Start PWM on TIM4 CH4 (sends the 10 us TRIG pulse every 60 ms)

lcd_init();                                                   // Initialise the LCD (must come AFTER TIM4 is started, because delay_us() needs it)
lcd_set_cursor(0, 0);                                         // Go to the first character of the top line
lcd_print("Hei Gizem");                                    // Boot message (replaced by the first measurement)
HAL_Delay(2000);
HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);                     // Start PWM on TIM1 CH1 (the buzzer pin PE9)
buzzer_set(0);                                                // Begin silent
alert_update(100);                                            // Start with the "far away" state (green LED on)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (new_pulse)                                            // Only act when the interrupt has delivered a new ultrasonic measurement
    {
      new_pulse = 0;                                          // Clear the flag so each result is handled once
      uint32_t us = pulse_us;                                 // Copy the echo pulse width so it cannot change while we use it
      uint32_t mm = us * 10 / 58;                             // Ultrasonic distance in mm: (us / 58) cm * 10
      alert_update(mm / 10);                                  // Update LEDs and buzzer from the ultrasonic distance in whole cm

      uint32_t sum = 0;                                       // Running total of ADC readings
      for (int i = 0; i < 16; i++)                            // Take 16 readings
      {
        HAL_ADC_Start(&hadc1);                                // Start one ADC conversion on PA3
        HAL_ADC_PollForConversion(&hadc1, 10);                // Wait until it finishes (give up after 10 ms)
        sum += HAL_ADC_GetValue(&hadc1);                      // Add the 12-bit result (0..4095) to the total
        HAL_ADC_Stop(&hadc1);                                 // Stop the ADC so the next start is clean
        HAL_Delay(IR_SAMPLE_GAP_MS);                          // Spread the samples over time so we cover a full Sharp update cycle
      }

      uint32_t avg = sum / 16;                                // Average raw ADC value
      uint32_t mv = avg * 3300 / 4095;                        // Convert to millivolts (4095 = 3.3 V)
      uint32_t ir_mm = ir_distance_mm(mv);                    // IR distance in mm from the first model (0 = out of range)
      sample_n++;                                             // Count this measurement

#if LOG_CSV                                                   // Logging mode: one CSV line per measurement
      int len = snprintf(buf, sizeof buf, "%lu,%lu,%lu\r\n",  // Template: n,pulse_us,adc
                         (unsigned long)sample_n,             // Running measurement number
                         (unsigned long)us,                   // Raw ultrasonic echo width in microseconds
                         (unsigned long)avg);                 // Raw averaged ADC value
#else                                                         // Normal mode: readable text
      int len = snprintf(buf, sizeof buf, "US: %lu.%lu cm | IR: %lu.%lu cm (%lu mV)\r\n",  // Template for a human reader
                         (unsigned long)(mm / 10),            // Ultrasonic whole cm
                         (unsigned long)(mm % 10),            // Ultrasonic tenth of a cm
                         (unsigned long)(ir_mm / 10),         // IR whole cm
                         (unsigned long)(ir_mm % 10),         // IR tenth of a cm
                         (unsigned long)mv);                  // IR voltage in mV
#endif                                                        // End of the logging/normal choice
      HAL_UART_Transmit(&huart3, (uint8_t*)buf, len, 100);    // Send the text to PuTTY

      char line[17];                                          // A 16-character LCD line plus the string terminator
      snprintf(line, sizeof line, "US:%4lu.%lu cm   ",        // Top line: ultrasonic distance padded to 16 characters
               (unsigned long)(mm / 10),                      // Whole centimetres
               (unsigned long)(mm % 10));                     // Tenth of a centimetre
      lcd_set_cursor(0, 0);                                   // Go to the start of the top line
      lcd_print(line);                                        // Print the top line

      if (ir_mm == 0)                                         // The IR voltage is outside the usable range
        snprintf(line, sizeof line, "IR: out of range");      // Show a message (exactly 16 characters)
      else                                                    // Otherwise show the IR distance
        snprintf(line, sizeof line, "IR:%4lu.%lu cm    ",     // Bottom line: IR distance padded to 16 characters
                 (unsigned long)(ir_mm / 10),                 // Whole centimetres
                 (unsigned long)(ir_mm % 10));                // Tenth of a centimetre
      lcd_set_cursor(1, 0);                                   // Go to the start of the bottom line
      lcd_print(line);                                        // Print the bottom line

      HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);             // Heartbeat: flip LD1 for each measurement
      HAL_Delay(200);                                         // Wait 200 ms (about 4 measurements per second in total)
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 108;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 108-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 108-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 59999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LCD_D7_Pin|LCD_D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, LCD_RS_Pin|LCD_E_Pin|LED_R_Pin|LED_Y_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LCD_D4_Pin|LCD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin LCD_D7_Pin LCD_D6_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LCD_D7_Pin|LCD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RS_Pin LCD_E_Pin LED_R_Pin LED_Y_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_E_Pin|LED_R_Pin|LED_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_G_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_G_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_D4_Pin LCD_D5_Pin */
  GPIO_InitStruct.Pin = LCD_D4_Pin|LCD_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)       // Called automatically by HAL whenever an edge is captured
{
  if (htim->Instance == TIM4 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)   // Only react to TIM4 channel 3 (our ECHO pin)
  {
    uint32_t t = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);   // Read the timer value that was frozen at the moment of the edge

    if (wait_fall == 0)                                            // We were waiting for the rising edge (echo starts)
    {
      ic_rise = t;                                                 // Remember the start time
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);   // Now trigger on the falling edge instead
      wait_fall = 1;                                               // Next edge will be the falling one
    }
    else                                                           // We were waiting for the falling edge (echo ends)
    {
      if (t >= ic_rise)                                            // Normal case: the timer did not wrap around between the edges
        pulse_us = t - ic_rise;                                    // Pulse width = end time minus start time
      else                                                         // The timer wrapped from 59999 back to 0 in between
        pulse_us = (t + 60000) - ic_rise;                          // Add one full period (60000 ticks) to compensate
      __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);     // Go back to waiting for a rising edge
      wait_fall = 0;                                               // Reset the state for the next measurement
      new_pulse = 1;                                               // Tell main() that a new result is ready
    }
  }
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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

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
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint32_t timestamp_ms;
    float temperature;
    float humidity;
    float pressure;
    uint16_t pot_raw;
    float threshold_temp;
    uint8_t alarm_active;
} SensorData;

typedef enum {
    PAGE_LIVE_DATA = 0,
    PAGE_MIN_MAX,
    PAGE_LOG_COUNT,
    PAGE_SYSTEM_INFO,
    PAGE_COUNT
} DisplayPage;

typedef struct {
    SensorData current;
    float temp_min, temp_max;
    float hum_min, hum_max;
    uint32_t log_count;
    uint32_t uptime_seconds;
    DisplayPage current_page;
    uint8_t alarm_armed;
    uint8_t alarm_triggered;
    uint8_t sd_card_ok;
    uint8_t bme280_ok;
    uint8_t bt_connected;
} SystemState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SSD1306_ADDR (0x3C << 1)
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
SystemState sys = {0};

/* Timing counters */
uint32_t tick_sensor  = 0;
uint32_t tick_display = 0;
uint32_t tick_sd      = 0;
uint32_t tick_bt      = 0;
uint32_t tick_second  = 0;

/* Button debounce */
uint8_t btn1_prev = 1, btn2_prev = 1;
uint8_t btn1_count = 0, btn2_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ---- SSD1306 OLED Driver ---- */
static uint8_t ssd1306_buf[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

/* 5x7 font, ASCII 32-90 */
static const uint8_t font5x7[][5] = {
    {0x00,0x00,0x00,0x00,0x00}, // ' '
    {0x00,0x00,0x5F,0x00,0x00}, // '!'
    {0x00,0x07,0x00,0x07,0x00}, // '"'
    {0x14,0x7F,0x14,0x7F,0x14}, // '#'
    {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
    {0x23,0x13,0x08,0x64,0x62}, // '%'
    {0x36,0x49,0x56,0x20,0x50}, // '&'
    {0x00,0x08,0x07,0x03,0x00}, // '\''
    {0x00,0x1C,0x22,0x41,0x00}, // '('
    {0x00,0x41,0x22,0x1C,0x00}, // ')'
    {0x2A,0x1C,0x7F,0x1C,0x2A}, // '*'
    {0x08,0x08,0x3E,0x08,0x08}, // '+'
    {0x00,0x80,0x70,0x30,0x00}, // ','
    {0x08,0x08,0x08,0x08,0x08}, // '-'
    {0x00,0x00,0x60,0x60,0x00}, // '.'
    {0x20,0x10,0x08,0x04,0x02}, // '/'
    {0x3E,0x51,0x49,0x45,0x3E}, // '0'
    {0x00,0x42,0x7F,0x40,0x00}, // '1'
    {0x72,0x49,0x49,0x49,0x46}, // '2'
    {0x21,0x41,0x49,0x4D,0x33}, // '3'
    {0x18,0x14,0x12,0x7F,0x10}, // '4'
    {0x27,0x45,0x45,0x45,0x39}, // '5'
    {0x3C,0x4A,0x49,0x49,0x31}, // '6'
    {0x41,0x21,0x11,0x09,0x07}, // '7'
    {0x36,0x49,0x49,0x49,0x36}, // '8'
    {0x46,0x49,0x49,0x29,0x1E}, // '9'
    {0x00,0x00,0x14,0x00,0x00}, // ':'
    {0x00,0x40,0x34,0x00,0x00}, // ';'
    {0x00,0x08,0x14,0x22,0x41}, // '<'
    {0x14,0x14,0x14,0x14,0x14}, // '='
    {0x00,0x41,0x22,0x14,0x08}, // '>'
    {0x02,0x01,0x59,0x09,0x06}, // '?'
    {0x3E,0x41,0x5D,0x59,0x4E}, // '@'
    {0x7C,0x12,0x11,0x12,0x7C}, // 'A'
    {0x7F,0x49,0x49,0x49,0x36}, // 'B'
    {0x3E,0x41,0x41,0x41,0x22}, // 'C'
    {0x7F,0x41,0x41,0x41,0x3E}, // 'D'
    {0x7F,0x49,0x49,0x49,0x41}, // 'E'
    {0x7F,0x09,0x09,0x09,0x01}, // 'F'
    {0x3E,0x41,0x41,0x51,0x73}, // 'G'
    {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
    {0x00,0x41,0x7F,0x41,0x00}, // 'I'
    {0x20,0x40,0x41,0x3F,0x01}, // 'J'
    {0x7F,0x08,0x14,0x22,0x41}, // 'K'
    {0x7F,0x40,0x40,0x40,0x40}, // 'L'
    {0x7F,0x02,0x1C,0x02,0x7F}, // 'M'
    {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
    {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
    {0x7F,0x09,0x09,0x09,0x06}, // 'P'
    {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
    {0x7F,0x09,0x19,0x29,0x46}, // 'R'
    {0x26,0x49,0x49,0x49,0x32}, // 'S'
    {0x03,0x01,0x7F,0x01,0x03}, // 'T'
    {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
    {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
    {0x3F,0x40,0x38,0x40,0x3F}, // 'W'
    {0x63,0x14,0x08,0x14,0x63}, // 'X'
    {0x03,0x04,0x78,0x04,0x03}, // 'Y'
    {0x61,0x59,0x49,0x4D,0x43}, // 'Z'
};

uint8_t SSD1306_Init(void)
{
    const uint8_t init_cmds[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00,
        0x40, 0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8,
        0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1, 0xDB,
        0x40, 0xA4, 0xA6, 0xAF
    };
    for (uint8_t i = 0; i < sizeof(init_cmds); i++) {
        uint8_t buf[2] = {0x00, init_cmds[i]};
        if (HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR, buf, 2, 100) != HAL_OK)
            return 0;
    }
    memset(ssd1306_buf, 0, sizeof(ssd1306_buf));
    return 1;
}

void SSD1306_Clear(void)
{
    memset(ssd1306_buf, 0, sizeof(ssd1306_buf));
}

void SSD1306_SetPixel(uint8_t x, uint8_t y, uint8_t on)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
    if (on) ssd1306_buf[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    else    ssd1306_buf[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
}

void SSD1306_WriteStr(uint8_t x, uint8_t y, const char *str)
{
    while (*str && x < SSD1306_WIDTH - 5) {
        uint8_t c = (uint8_t)*str;
        if (c >= 32 && c <= 90) {
            for (int i = 0; i < 5; i++) {
                uint8_t col = font5x7[c - 32][i];
                for (int bit = 0; bit < 7; bit++) {
                    SSD1306_SetPixel(x + i, y + bit, (col >> bit) & 1);
                }
            }
        }
        x += 6;
        str++;
    }
}

void SSD1306_Update(void)
{
    for (uint8_t page = 0; page < 8; page++) {
        uint8_t cmd[2];
        cmd[0] = 0x00; cmd[1] = 0xB0 + page;
        HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR, cmd, 2, 100);
        cmd[1] = 0x00;
        HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR, cmd, 2, 100);
        cmd[1] = 0x10;
        HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR, cmd, 2, 100);

        uint8_t data[SSD1306_WIDTH + 1];
        data[0] = 0x40;
        memcpy(&data[1], &ssd1306_buf[page * SSD1306_WIDTH], SSD1306_WIDTH);
        HAL_I2C_Master_Transmit(&hi2c1, SSD1306_ADDR, data, SSD1306_WIDTH + 1, 100);
    }
}

/* UART printf redirect */
int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* Page name helper */
static const char *page_name(DisplayPage p)
{
    switch (p) {
        case PAGE_LIVE_DATA:   return "LIVE";
        case PAGE_MIN_MAX:     return "MINMAX";
        case PAGE_LOG_COUNT:   return "LOG";
        case PAGE_SYSTEM_INFO: return "INFO";
        default:               return "?";
    }
}

/* Read internal temperature sensor (ADC channel 16)
 * Formula from datasheet: T(C) = ((V25 - Vsense) / Avg_Slope) + 25
 *   V25 = 1.43V typical, Avg_Slope = 4.3 mV/C typical
 *   Vsense = (ADC_raw / 4095) * 3.3V
 */
float ReadInternalTemp(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    float vsense = ((float)raw / 4095.0f) * 3.3f;
    return ((1.43f - vsense) / 0.0043f) + 25.0f;
}

/* Read potentiometer on PA0 (ADC channel 0)
 * Maps 0-4095 to 15.0-50.0 degrees C threshold
 */
float ReadThreshold(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t raw = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    sys.current.pot_raw = raw;
    return 15.0f + ((float)raw / 4095.0f) * 35.0f;
}

/* Debounced button polling */
void PollButtons(void)
{
    uint8_t b1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    uint8_t b2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

    /* Button 1: cycle display page */
    if (btn1_prev == 1 && b1 == 0) {
        btn1_count++;
        if (btn1_count > 3) {
            sys.current_page = (DisplayPage)((sys.current_page + 1) % PAGE_COUNT);
            btn1_count = 0;
            printf("[BTN] Page -> %s\r\n", page_name(sys.current_page));
        }
    } else {
        btn1_count = 0;
    }
    btn1_prev = b1;

    /* Button 2: arm/disarm alarm */
    if (btn2_prev == 1 && b2 == 0) {
        btn2_count++;
        if (btn2_count > 3) {
            sys.alarm_armed = !sys.alarm_armed;
            btn2_count = 0;
            printf("[BTN] Alarm %s\r\n", sys.alarm_armed ? "ARMED" : "DISARMED");
        }
    } else {
        btn2_count = 0;
    }
    btn2_prev = b2;
}

/* Alarm check with LED/relay/buzzer control */
void Alarm_Check(SystemState *state)
{
    if (!state->alarm_armed) {
        state->alarm_triggered = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); /* Relay off */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); /* Buzzer off */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); /* Green off */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); /* Yellow off */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);  /* Red off */
        return;
    }

    float margin = state->current.threshold_temp - state->current.temperature;

    if (state->current.temperature > state->current.threshold_temp) {
        state->alarm_triggered = 1;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   /* Relay on */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   /* Buzzer on */
    } else {
        state->alarm_triggered = 0;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    }

    /* Status LEDs */
    if (state->alarm_triggered) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);    /* Red */
    } else if (margin < 5.0f && margin > 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   /* Yellow */
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   /* Green */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
    }
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Relay and buzzer off at startup */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

  /* Green LED on during init */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);

  /* Initialize OLED */
  uint8_t oled_ok = SSD1306_Init();
  if (oled_ok) {
      SSD1306_Clear();
      SSD1306_WriteStr(0, 0, "MULTI-SENSOR");
      SSD1306_WriteStr(0, 10, "DATA LOGGER");
      SSD1306_WriteStr(0, 25, "INITIALIZING...");
      SSD1306_Update();
      HAL_Delay(1000);
  }

  printf("\r\n[INIT] Multi-Sensor Data Logger\r\n");
  printf("[INIT] OLED: %s\r\n", oled_ok ? "OK" : "FAIL");
  printf("[INIT] Step 1: Internal temp + OLED + GPIO\r\n");

  /* Read initial temperature */
  float t0 = ReadInternalTemp();
  sys.current.temperature = t0;
  sys.current.humidity = 0.0f;
  sys.current.pressure = 0.0f;
  sys.temp_min = sys.temp_max = t0;
  sys.hum_min = sys.hum_max = 0.0f;

  sys.alarm_armed = 1;
  sys.bme280_ok = 0;   /* No external sensor in Step 1 */
  sys.sd_card_ok = 0;
  sys.bt_connected = 0;
  sys.log_count = 0;

  printf("[INIT] Internal temp = %.1f C\r\n", t0);
  printf("[INIT] All peripherals initialized\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    /* Poll buttons every ~10 ms */
    PollButtons();

    /* Heartbeat: toggle onboard LED every 500 ms
     * WeAct Blue Pill: PB2 (active high)
     * Generic Blue Pill: PC13 (active low)
     */
    static uint32_t tick_led = 0;
    if (now - tick_led >= 500) {
        tick_led = now;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }

    /* Sensor read every 100 ms */
    if (now - tick_sensor >= 100) {
        tick_sensor = now;
        sys.current.timestamp_ms = now;

        sys.current.temperature = ReadInternalTemp();
        sys.current.threshold_temp = ReadThreshold();

        /* Update min/max */
        if (sys.current.temperature < sys.temp_min)
            sys.temp_min = sys.current.temperature;
        if (sys.current.temperature > sys.temp_max)
            sys.temp_max = sys.current.temperature;

        /* Check alarm */
        sys.current.alarm_active = sys.alarm_triggered;
        Alarm_Check(&sys);
    }

    /* Display every 500 ms */
    if (now - tick_display >= 500) {
        tick_display = now;

        /* Update OLED */
        char buf[22];
        SSD1306_Clear();

        snprintf(buf, sizeof(buf), "T: %.1f C", sys.current.temperature);
        SSD1306_WriteStr(0, 0, buf);

        snprintf(buf, sizeof(buf), "THR: %.1f C", sys.current.threshold_temp);
        SSD1306_WriteStr(0, 10, buf);

        snprintf(buf, sizeof(buf), "POT: %u", sys.current.pot_raw);
        SSD1306_WriteStr(0, 20, buf);

        snprintf(buf, sizeof(buf), "UP: %lus  %s", sys.uptime_seconds, page_name(sys.current_page));
        SSD1306_WriteStr(0, 35, buf);

        if (sys.alarm_triggered)
            SSD1306_WriteStr(0, 50, "** ALARM **");
        else if (sys.alarm_armed)
            SSD1306_WriteStr(0, 50, "ARMED - OK");
        else
            SSD1306_WriteStr(0, 50, "DISARMED");

        SSD1306_Update();
    }

    /* SD card stub every 1 second (Step 5: SD_Logger_Write here) */
    if (now - tick_sd >= 1000) {
        tick_sd = now;
    }

    /* Bluetooth stub every 2 seconds (Step 4: BT_SendJSON here) */
    if (now - tick_bt >= 2000) {
        tick_bt = now;
    }

    /* Uptime counter */
    if (now - tick_second >= 1000) {
        tick_second = now;
        sys.uptime_seconds++;
        printf("[TICK] uptime=%lus  page=%s\r\n",
            sys.uptime_seconds, page_name(sys.current_page));
    }

    HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* WeAct Blue Pill onboard LED on PB2 (not in CubeMX, init manually) */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

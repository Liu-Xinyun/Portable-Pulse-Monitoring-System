/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "gpio.h"
#include "usart.h"
#include "adc.h"
#include "i2c.h"
#include "OLED12864.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include <stdio.h>
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define KEY1        HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_4)   
#define KEY3        HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3)  
#define KEY4        HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)  

#define  KEY1_PRES 1;
#define  KEY3_PRES 3;
#define  KEY4_PRES 4;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
typedef enum { MEASURING1,MEASURING2,STANDBY,WARNING } AppState; //几种状态模式
volatile AppState state = STANDBY;
int measuring_flag = 0;          // 是否正在测量中
int reset_flag = 1;              // 是否重置标志
int end_flag = 0;                // 是否测量结束
uint32_t delta = 0;              //两次心跳间隔时间
const int RATE_SIZE = 4;         // 平均心率计算窗口大小
int rates[RATE_SIZE];            // 存储最近4个心率值
int rateSpot = 0;                // 当前存储心率的位置索引
int beatAvg = 0;                 // 平均BPM（MEASURING2中计算）
long lastBeat = 0;               // 上一次心跳的时间
int beatcount = 0;               // 当前心跳数（MeASURING1计数）
int Total = 0;                   // 60秒内的总心跳数
float beatsPerMinute = 0;        // 当前计算出瞬时BPM值
long start;                      //60s计时的开始时间
char Data[4];                    //储存数值
uint8_t flag_KS = 1;             //按键按下检测
uint8_t	KeyV =0;                 //哪一个按键被按下
long beep_start=0;               //蜂鸣器开始工作时刻
long result0;                    //存储读取的ADC值
uint8_t mode=1;                  // 模式选择（1为串口，2为蓝牙）
int highwarning=110;             // 高心率报警阈值
int lowwarning=50;               // 低心率报警阈值
int warningmode=0;               // 当前警告模式编号
uint8_t TxBuf[16];               // 串口发送缓冲区
uint8_t RxBuf[32];               // 串口接收缓冲区

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void OLED_Task(void);                // OLED 显示任务
uint8_t KEY_Scan(uint8_t mode);      // 按键扫描
void KEY_Task(void);                 // 按键处理任务
void Uart_task(void);                // 串口任务
void uart_fast_set_baudrate(uint32_t baudrate); // 快速修改串口波特率
void send_bluetooth_text(float voltage,int BMP); // 通过蓝牙发送数据
void beep_task(void);                // 蜂鸣器任务
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM4 ) {
    uint32_t now = HAL_GetTick();        // 获取当前时间（ms）
    uint32_t diff_ms = now - lastBeat;   // 当前与上次心跳的时间差
    if(diff_ms < 350) return;            // 过滤高频干扰（小于350ms认为无效）

    delta = diff_ms;                     // 记录心跳周期
    lastBeat = now;                      // 更新上一次心跳时间

    if(state == MEASURING1) {           // 模式1：60秒累计计次
      if (reset_flag) {                 // 第一次检测心跳
        measuring_flag = 1;
        reset_flag     = 0;
        end_flag       = 0;
        start          = now;
        beatcount      = 1;
      } else if (end_flag && delta >= 3000) { // 测量已结束后3秒再次启动
        measuring_flag = 1;
        end_flag       = 0;
        start          = now;
        beatcount      = 1;
      } else if (measuring_flag) {
        if ((now - start) < 60000 ) {   // 测量60秒内
          beatcount++;
        } else {                        // 超过60秒，记录测量结果
          measuring_flag = 0;
          end_flag       = 1;
          Total          = beatcount;
          beatcount      = 0;
          beep_start     = now;
        }
      }
    } else if(state == MEASURING2) {    // 模式2：即时计算平均心率
      measuring_flag = 1;
      reset_flag = 0;
      end_flag = 0;
      beatsPerMinute = 60 / (delta / 1000.0); // BPM = 60 / (心跳周期秒)

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (int)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (int x = 0 ; x < RATE_SIZE ; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;           // 取最近4个心率平均值
      }
    }
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();             
  HAL_TIM_Base_Start(&htim4);    
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      OLED_Task();              // OLED刷新显示
      KEY_Task();               // 按键处理
      Uart_task();              // 串口采集发送
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  
  /* USER CODE END 3 */
  }
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
  sConfig.Channel = ADC_CHANNEL_2;
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
  hi2c1.Init.ClockSpeed = 100000;
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BELL_GPIO_Port, BELL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SW3_Pin SW1_Pin SW4_Pin */
  GPIO_InitStruct.Pin = SW3_Pin|SW1_Pin|SW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BELL_Pin */
  GPIO_InitStruct.Pin = BELL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BELL_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* 串口采集任务，按模式选择输出格式 */
void Uart_task(void){
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    result0 = HAL_ADC_GetValue(&hadc1);
    static uint32_t last_vofa_send = 0;

    if(mode==1){ // 串口模式
        if (HAL_GetTick() - last_vofa_send >= 20) {
            last_vofa_send = HAL_GetTick();
            float voltage = result0 * 3.3f / 4095.0f;
            uart_fast_set_baudrate(115200);
            char msg[64];
            if(state==MEASURING2){
                int len = sprintf(msg, "voltage=%.2f  BPM=%d\r\n", voltage, beatAvg);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
            }else if(state==MEASURING1){
                int len = sprintf(msg, "voltage=%.2f  BPM=%d\r\n", voltage, Total);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100);
            }else{
                sprintf(msg, "%.2f\r\n", voltage);
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
            }
        }
    }else if(mode==2){ // 蓝牙模式
        uart_fast_set_baudrate(9600);
        float voltage = result0 * 3.3f / 4095.0f;
        if(state==MEASURING1)
            send_bluetooth_text(voltage, Total);
        else if(state==MEASURING2)
            send_bluetooth_text(voltage, beatAvg);
    }
    HAL_Delay(20);
}



void OLED_Task(void){
    if (state==WARNING){
			if(warningmode==0){
			  OLED_P16X32Str(0,0,"High=110");
        OLED_P16X32Str(0,4,"Low=50");
			}else if(warningmode==1){
			  OLED_P16X32Str(0,0,"High=90");
        OLED_P16X32Str(0,4,"Low=60");
			}else if(warningmode==2){
			  OLED_P16X32Str(0,0,"High=100");
        OLED_P16X32Str(0,4,"Low=60");
			}else if(warningmode==3){
			  OLED_P16X32Str(0,0,"High=110");
        OLED_P16X32Str(0,4,"Low=60");
			}else if(warningmode==4){
			  OLED_P16X32Str(0,0,"High=90");
        OLED_P16X32Str(0,4,"Low=50");
			}else if(warningmode==5){
			  OLED_P16X32Str(0,0,"High=100");
        OLED_P16X32Str(0,4,"Low=50");
			}
		}
    if (state == MEASURING2 && reset_flag == 1){
        OLED_P16X32Str(0,0,"Mode 2  ");
        OLED_P16X32Str(0,4,"Ready   ");
    }else if (state == MEASURING2 && measuring_flag == 1){
        OLED_P16X32Str(0,0,"BPM:    ");
        sprintf(Data, "%d", beatAvg);
        OLED_P16X32Str(64,0,Data);
        if (beatAvg >= lowwarning && beatAvg <= highwarning ) {
            OLED_P16X32Str(0,4,"Normal  ");
            HAL_GPIO_WritePin(BELL_GPIO_Port, BELL_Pin, GPIO_PIN_RESET);
        }
        else if (beatAvg > highwarning && beatAvg <225 ) {
            OLED_P16X32Str(0,4,"Too Fast");
            HAL_GPIO_WritePin(BELL_GPIO_Port, BELL_Pin, GPIO_PIN_SET);
        }
        else if (beatAvg > 20 && beatAvg <lowwarning ){
            OLED_P16X32Str(0,4,"Too Slow");
            HAL_GPIO_WritePin(BELL_GPIO_Port, BELL_Pin, GPIO_PIN_SET);
        } else{
           OLED_P16X32Str(0,4,"        ");
           HAL_GPIO_WritePin(BELL_GPIO_Port, BELL_Pin, GPIO_PIN_RESET);
        }
    }else if (state == MEASURING1 && reset_flag == 1){
        OLED_P16X32Str(0,0,"Mode 1  ");
        OLED_P16X32Str(0,4,"Ready   ");
    }else if (state == MEASURING1 && measuring_flag == 1){
        OLED_P16X32Str(0,0,"Wait... ");
        OLED_P16X32Str(0,4,"cnt:    ");
        sprintf(Data, "%d", beatcount);
        OLED_P16X32Str(64,4,Data);
    }else if (state == MEASURING1 && end_flag == 1){
        OLED_P16X32Str(0,0,"BPM:    ");
        sprintf(Data, "%d", Total);
        OLED_P16X32Str(64,0,Data);
        if (Total >= lowwarning && Total <= highwarning ) {
            OLED_P16X32Str(0,4,"Normal  ");
        }
        else if (Total > highwarning && Total <225 ) {
            OLED_P16X32Str(0,4,"Too Fast");
            beep_task();
        }
        else if (Total > 20 && Total <lowwarning ){
            OLED_P16X32Str(0,4,"Too Slow");
            beep_task();
        }  
    } 
}


/* 按键扫描函数，检测按键是否按下 */
uint8_t KEY_Scan(uint8_t mode)
{
	if(KEY1 ==0)
	{
		return KEY1_PRES;
	}
	else if(KEY3 ==0)
	{
		return KEY3_PRES;
	}
	else if(KEY4 ==0)
	{
		return KEY4_PRES;
	}
	else{
		return 0;
	}
}

void KEY_Task(void){
    if(flag_KS){
		KeyV = KEY_Scan(0);				
		switch (KeyV){ 
      case 1 :   // KEY1：测量模式可切换传输模式，WARINING模式可更换报警阈值
				OLED_Clear();
			  if(mode==1&&(state!=MEASURING1)&&(state!=MEASURING2))
				state=WARNING;
				if(state==MEASURING1||state==MEASURING2){
				if(mode==1)
					mode=2;
				else if(mode==2)
					mode=1; 
        }
				else if(state==WARNING){
					if(warningmode==0){
						highwarning=90;
						lowwarning=60;
						warningmode=1;
          }else if(warningmode==1){
					  highwarning=100;
					  lowwarning=60;
					  warningmode=2;
				  }else if(warningmode==2){
					  highwarning=110;
					  lowwarning=60;
					  warningmode=3;
				  }else if(warningmode==3){
					  highwarning=90;
					  lowwarning=50;
					  warningmode=4;
				  }else if(warningmode==4){
				  	highwarning=100;
				  	lowwarning=50;
				  	warningmode=5;
				  }else if(warningmode==5){
				  	highwarning=110;
					  lowwarning=50;
					  warningmode=0;
				  }
        }
				break;
			case 3 : //KEY3：进入测量模式1
            OLED_Clear();
            state = MEASURING1;
            reset_flag = 1;
            measuring_flag = 0;
            end_flag = 0;
            beatcount = 0;  
			break;
			case 4: //KEY4：进入测量模式2
            OLED_Clear();
			      state = MEASURING2;
            reset_flag = 1;
            measuring_flag = 0;
            end_flag = 0;
			break;
            default:	 
			break;
    }		
	}
}

/* 发送蓝牙格式化字符串 */
void send_bluetooth_text(float voltage, int BPM) {
    char msg[64];
    int len = sprintf(msg, "voltage=%.2f  BPM=%d\r\n", voltage, BPM);
    if (HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 100) != HAL_OK) {
        Error_Handler();
    }
}

int _write(int fd, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

/* UART 快速设置波特率 */
void uart_fast_set_baudrate(uint32_t baudrate) {
    uint32_t usartdiv = HAL_RCC_GetPCLK2Freq() / baudrate;
    MODIFY_REG(USART1->BRR, 0xFFFF, usartdiv);
}

/* 蜂鸣器任务（控制报警时长） */
void beep_task(void){
  if((HAL_GetTick()-beep_start)>1000){
    HAL_GPIO_WritePin(BELL_GPIO_Port, BELL_Pin, GPIO_PIN_RESET);
    return;
  }
  HAL_GPIO_WritePin(BELL_GPIO_Port, BELL_Pin, GPIO_PIN_SET);

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

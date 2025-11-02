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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "delay.h"
#include "gpio.h"
#include "icm20948_reg.h"
#include "main.h"
#include "motor.h"
#include "stm32f1xx_hal.h"
#include "sys.h"
#include "tim.h"
#include "usart.h"
#include "usart_x.h"

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
extern u8 Usart1_Receive_buf[1];
volatile int Count = 0;
uint16_t adc_val;
uint32_t voltage;
float Velocity_KP = 500.f, Velocity_KI = 50.f;
float Akm_Along_Distance_KP = 0.002f, Akm_Along_Distance_KD = 1000.245f,
      Akm_Along_Distance_KI = 0.00001f;
float Distance_KP = 0.0015f, Distance_KD = 0.25557f, Distance_KI = 0.00001f;
float Follow_KP_Akm = 0.02f, Follow_KD_Akm = 0.0182f, Follow_KI_Akm = -0.0001f;
float Move_X = 0, Move_Y = 0, Move_Z = 0;
float RC_Velocity = 350.f, RC_Turn_Velocity = 0.f;
uint8_t Flag_Direction = 0, PID_Send = 0, Flag_Left = 0, Flag_Right = 0;
uint8_t Receive_Data[12];
uint8_t Lidar_Success_Receive_flag = 0;
int PointDataProcess_count = 0, test_once_flag = 0, Dividing_point = 0;
uint8_t one_lap_data_success_flag = 0;
int Servo_PWM = SERVO_INIT;

pIMUInterface_t imu = &UserICM20948;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void test_move_state(void) {
  // Many embedded printf implementations (newlib-nano) omit float support
  // by default. Print scaled integer values (x1000) to observe the numbers
  // int mx = (int)(Move_X * 1000.0f);
  // int mz = (int)(Move_Z * 1000.0f);
  int PWMA_int = (int)(PWMA);
  int PWMB_int = (int)(PWMB);
  int motorA_target = (int)(MOTOR_A.Target * 1000.0f);
  int motorA_encoder = (int)(MOTOR_A.Encoder * 1000.0f);
  int motorA_pwm = (int)(MOTOR_A.Motor_Pwm);
  int motorB_target = (int)(MOTOR_B.Target * 1000.0f);
  int motorB_encoder = (int)(MOTOR_B.Encoder * 1000.0f);
  int motorB_pwm = (int)(MOTOR_B.Motor_Pwm);
  // printf("Move_X: %d (x1e-3), Move_Z: %d (x1e-3), Count: %d\r\n", mx, mz,
  //        Count);
  // printf("voltage: %u, TurnOff: %d\r\n", (unsigned)voltage,
  // Turn_Off(voltage));
  printf("MOTOR_A encoder: %d, MOTOR_B encoder: %d\r\n", motorA_encoder,
         motorB_encoder);
  printf("MOTOR_A target: %d (x1e-3), pwm: %d; MOTOR_B target: %d (x1e-3), "
         "pwm: %d\r\n",
         motorA_target, motorA_pwm, motorB_target, motorB_pwm);
  printf("PWMA(REG CCR2): %d, PWMB(REG CCR1): %d\r\n", PWMA_int, PWMB_int);
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
  delay_init(72);
  setvbuf(stdout, NULL, _IONBF, 0);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // PWMB (或根据你连线为 PWMA)
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // PWMA (或根据你连线为 PWMB)
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_UART_Receive_IT(&huart1, Usart1_Receive_buf, sizeof(Usart1_Receive_buf));
  while (imu->Init()) {
    printf("imu initing...\r\n");
  }
  Motor_Init();
  MX_ADC1_Init();                      // 先初始化ADC1
  HAL_ADCEx_Calibration_Start(&hadc1); // 然后再进行校准

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    adc_val = Get_Adc(11);
    voltage = adc_val * 33 * 11 * 100 / 4096 / 10;
    test_move_state();
    delay_ms(1000);
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
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

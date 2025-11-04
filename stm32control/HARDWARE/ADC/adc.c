#include "h_adc.h"
#include "stm32f1xx_hal.h"

// ADC1 初始化
void ADC1_Init(void) {
  __HAL_RCC_ADC1_CLK_ENABLE();  // ADC1 时钟使能
  __HAL_RCC_GPIOC_CLK_ENABLE(); // GPIOC 时钟使能

  // 1. 配置 GPIOC_PIN1 为模拟输入
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // 2. 配置 ADC1
  static ADC_HandleTypeDef hadc1 = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE; // 单通道
  hadc1.Init.ContinuousConvMode = DISABLE;    // 非连续模式
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START; // 软件触发
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;       // 右对齐
  hadc1.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc1);

  // 3. 配置通道（对应 PC1 是 ADC_CHANNEL_11）
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  // 4. 校准 ADC（HAL 里只需要调用一次）
  HAL_ADCEx_Calibration_Start(&hadc1);
}

// 单通道 ADC 读取
uint16_t Get_Adc1(uint32_t channel) {
  static ADC_HandleTypeDef hadc1 = {0};
  hadc1.Instance = ADC1;

  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  uint16_t value = HAL_ADC_GetValue(&hadc1);
  HAL_ADC_Stop(&hadc1);

  return value;
}

// 连续读取 ADC 并取平均值
uint16_t Get_Adc_Average(uint32_t channel, uint8_t count) {
  uint32_t temp_val = 0;
  for (uint8_t i = 0; i < count; i++) {
    temp_val += Get_Adc1(channel);
    HAL_Delay(5); // 替代 delay_ms
  }
  return (uint16_t)(temp_val / count);
}

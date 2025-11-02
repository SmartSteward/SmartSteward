#ifndef __ADC_H
#define __ADC_H

#include "sys.h"

void ADC1_Init(void);
u16 Get_Adc1(uint32_t channel);
u16 Get_adc_Average(uint32_t channel, uint8_t count);

#endif

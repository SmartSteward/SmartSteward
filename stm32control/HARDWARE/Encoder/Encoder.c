#include "Encoder.h"
#include "stm32f1xx_hal.h"

/*------------------ TIMx read encoder------------------*/
int Read_Encoder(uint8_t TIMX) {
  int Encoder_TIM = 0;
  switch (TIMX) {
  case 3:
    /* TIMx->CNT is a 16-bit hardware counter. When the encoder runs in
       the reverse direction CNT wi`ll underflow and hold a large unsigned
       value (e.g. 0xFFFE). Cast to int16_t so the two's complement value
       is interpreted as a negative delta. */
    Encoder_TIM = (int16_t)TIM3->CNT;
    TIM3->CNT = 0;
    break;
  case 5:
    /* Same signed interpretation for TIM5 */
    Encoder_TIM = (int16_t)TIM5->CNT;
    TIM5->CNT = 0;
    break;
  default:
    Encoder_TIM = 0;
  }
  return Encoder_TIM;
}

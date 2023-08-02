#include "board.h"
#include "ad9269.h"
#include "cmsis_gcc.h"
#include "keys.h"
#include "lmx2572_legacy.h"
#include "main.h"
#include "spi.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"
#include "timers.h"
#include <string.h>

AD9269 ad9269_pins;

void BOARD_InitAD9269() {
  ad9269_pins.EN_Port = ADC_CS_GPIO_Port;
  ad9269_pins.EN_Pin = ADC_CS_Pin;
  ad9269_pins.SCK_Port = ADC_SCK_GPIO_Port;
  ad9269_pins.SCK_Pin = ADC_SCK_Pin;
  ad9269_pins.SDA_Port = ADC_DIO_GPIO_Port;
  ad9269_pins.SDA_Pin = ADC_DIO_Pin;
  ad9269_pins.SLEEP_Port = ADC_SLEEP_GPIO_Port;
  ad9269_pins.SLEEP_Pin = ADC_SLEEP_Pin;
  AD9269_Init(&ad9269_pins);
}

LMX2572L_Pins lmx2572_pins;
void BOARD_InitLMX2572() {
  lmx2572_pins.CS_Port = PLL_CS_GPIO_Port;
  lmx2572_pins.CS_Pin = PLL_CS_Pin;
  lmx2572_pins.ENABLE_Port = PLL_ENABLE_GPIO_Port;
  lmx2572_pins.ENABLE_Pin = PLL_ENABLE_Pin;
  lmx2572_pins.SCK_Port = PLL_SCK_GPIO_Port;
  lmx2572_pins.SCK_Pin = PLL_SCK_Pin;
  lmx2572_pins.SDI_Port = PLL_SDI_GPIO_Port;
  lmx2572_pins.SDI_Pin = PLL_SDI_Pin;
  LMX2572L_Init(&lmx2572_pins);
  LMX2572L_SetFrequency(40e6);
}

void BOARD_Delay(int n) {
  for (int i = 0; i < n; i++) {
    __NOP();
  }
}

void BOARD_ReadRawADCDataSync(int16_t *data) {
  // memset(data, 0, 4096 * 2);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  TIM_DelayUs(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  TIM_DelayUs(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
  TIM_DelayUs(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_SPI_Receive(&hspi2, (uint8_t *)data, 4096, 1000);
}
#include "board.h"
#include "ad9269.h"
#include "cmsis_gcc.h"
#include "keys.h"
#include "lmx2572_legacy.h"
#include "main.h"
#include "serial.h"
#include "spi.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"
#include "stm32h7xx_hal_tim.h"
#include "tim.h"
#include "timers.h"
#include <stdint.h>
#include <stdio.h>
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
  LMX2572L_SetFrequency(BOARD_FREQ);
}

void BOARD_Delay() {
  for (int i = 0; i < 20; i++) {
    __NOP();
  }
}

#define WRITE(pin, state)                                                      \
  HAL_GPIO_WritePin(FPGA_##pin##_GPIO_Port, FPGA_##pin##_Pin,                  \
                    (state ? GPIO_PIN_SET : GPIO_PIN_RESET))

void BOARD_ResetFPGA() {
  WRITE(RESET, 0);
  BOARD_Delay();
  WRITE(RESET, 1);
  BOARD_Delay();
}

void BOARD_ReadRawADCDataSync(int16_t *data) {
  WRITE(READST, 0);
  BOARD_Delay();
  WRITE(READST, 1);
  BOARD_Delay();
  int startTick = HAL_GetTick();
  while (HAL_GPIO_ReadPin(FPGA_BUSY_GPIO_Port, FPGA_BUSY_Pin) ==
         GPIO_PIN_RESET) {
    if (HAL_GetTick() - startTick > 1000) {
      printf("FPGA timeout\n");
      return;
    }
  }
  WRITE(READST, 0);
  HAL_SPI_Receive(&hspi2, (uint8_t *)data, 4096, 1000);
}

void BOARD_WriteSPI(uint16_t addr, uint16_t data) {
  WRITE(CS, 0);
  WRITE(SCLK, 0);
  BOARD_Delay();
  for (int i = 15; i >= 0; i--) {
    WRITE(SDA, (addr >> i) & 1);
    BOARD_Delay();
    WRITE(SCLK, 1);
    BOARD_Delay();
    WRITE(SCLK, 0);
  }
  for (int i = 15; i >= 0; i--) {
    WRITE(SDA, (data >> i) & 1);
    BOARD_Delay();
    WRITE(SCLK, 1);
    BOARD_Delay();
    WRITE(SCLK, 0);
  }
  BOARD_Delay();
  WRITE(CS, 1);
  BOARD_Delay();
}

uint32_t BOARD_LastTriggerTick = 0;
void BOARD_SetTriggerFrequency(double freq) {
  uint32_t counter = (freq == 0) ? 0 : round(BOARD_FREQ / freq - 30);
  if (counter == BOARD_LastTriggerTick) {
    return;
  }
  BOARD_LastTriggerTick = counter;
  BOARD_WriteSPI(0x00, counter >> 16);
  BOARD_WriteSPI(0x01, counter);
}

uint64_t BOARD_LastFreqA = 0;
void BOARD_SetFrequencyA(double freq) {
  uint64_t ftw = (uint64_t)round(freq / BOARD_FREQ * (1ull << 32));
  if (ftw == BOARD_LastFreqA) {
    return;
  }
  BOARD_LastFreqA = ftw;
  BOARD_WriteSPI(0x03, ftw >> 16);
  BOARD_WriteSPI(0x04, ftw);
}

uint64_t BOARD_LastFreqB = 0;
void BOARD_SetFrequencyB(double freq) {
  uint64_t ftw = (uint64_t)round(freq / BOARD_FREQ * (1ull << 32));
  if (ftw == BOARD_LastFreqB) {
    return;
  }
  BOARD_LastFreqB = ftw;
  BOARD_WriteSPI(0x06, ftw >> 16);
  BOARD_WriteSPI(0x07, ftw);
}

uint16_t BOARD_LastOutput = 0;
void BOARD_SetOutput(int mode_a, int mode_b) {
  uint16_t output = (mode_a << 8) | mode_b;
  if (output == BOARD_LastOutput) {
    return;
  }
  BOARD_LastOutput = output;
  BOARD_WriteSPI(0x08, output);
}

uint64_t BOARD_LastPhaseA = 0;
void BOARD_SetPhaseA(double phase_deg) {
  uint64_t pow = (uint64_t)round(phase_deg / 360 * (1ull << 32));
  if (pow == BOARD_LastPhaseA) {
    return;
  }
  BOARD_LastPhaseA = pow;
  BOARD_WriteSPI(0x09, pow >> 16);
  BOARD_WriteSPI(0x0A, pow);
}

uint64_t BOARD_LastPhaseB = 0;
void BOARD_SetPhaseB(double phase_deg) {
  uint64_t pow = (uint64_t)round(phase_deg / 360 * (1ull << 32));
  if (pow == BOARD_LastPhaseB) {
    return;
  }
  BOARD_LastPhaseB = pow;
  BOARD_WriteSPI(0x0B, pow >> 16);
  BOARD_WriteSPI(0x0C, pow);
}
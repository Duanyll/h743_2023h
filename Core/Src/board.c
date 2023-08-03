#include "board.h"
#include "ad9269.h"
#include "cmsis_gcc.h"
#include "keys.h"
#include "lmx2572_legacy.h"
#include "main.h"
#include "serial.h"
#include "spi.h"
#include "stm32h743xx.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_spi.h"
#include "timers.h"
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


void BOARD_SetTriggerFrequency(double freq) {
  uint32_t counter = (uint32_t)round(BOARD_FREQ / freq - 50);
  printf("counter: %d\n", counter);
  BOARD_WriteSPI(0x00, counter >> 16);
  BOARD_WriteSPI(0x01, counter);
}

void BOARD_SetFrequencyA(double freq) {
  uint64_t ftw = (uint64_t)round(freq / BOARD_FREQ * (1ull << 31));
  // printf("ftw: %llu\n", ftw);
  // BOARD_WriteSPI(0x02, ftw >> 32);
  BOARD_WriteSPI(0x03, ftw >> 16);
  BOARD_WriteSPI(0x04, ftw);
}

void BOARD_SetFrequencyB(double freq) {
  uint64_t ftw = (uint64_t)round(freq / BOARD_FREQ * (1ull << 31));
  // BOARD_WriteSPI(0x05, ftw >> 32);
  BOARD_WriteSPI(0x06, ftw >> 16);
  BOARD_WriteSPI(0x07, ftw);
}

void BOARD_SetOutput(int mode_a, int mode_b) {
  BOARD_WriteSPI(0x08, mode_a << 8 | mode_b);
}

void BOARD_SetPhaseA(double phase_deg) {
  uint64_t pow = (uint64_t)round(phase_deg / 360 * (1ull << 32));
  BOARD_WriteSPI(0x09, pow >> 16);
  BOARD_WriteSPI(0x0A, pow);
}

void BOARD_SetPhaseB(double phase_deg) {
  uint64_t pow = (uint64_t)round(phase_deg / 360 * (1ull << 32));
  BOARD_WriteSPI(0x0B, pow >> 16);
  BOARD_WriteSPI(0x0C, pow);
}
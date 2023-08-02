#include "app.h"
#include "cgs.h"
#include "keys.h"
#include "led.h"
#include "lmx2572_legacy.h"
#include "retarget.h"
#include "serial.h"
#include "signal.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_uart.h"
#include "timers.h"
#include <stdint.h>
#include <stdio.h>


KEYS_Pins keys_pins;

UART_RxBuffer com_buf;
UART_HandleTypeDef *computer;
int16_t ad_data[4096];

float fft_buffer[4096 * 2];
float mag_buffer[4096];

void APP_RunFrequencyCounter() {
  printf("Running frequency counter\n");
  double bases[] = {43e6, 47e6, 53e6, 57e6};
  double base_coord[4];
  SIGNAL_FFTBufferF32 buffer = {
      .fftBuffer = fft_buffer,
      .magBuffer = mag_buffer,
  };
  for (int i = 0; i < 4; i++) {
    LMX2572L_SetFrequency(bases[i]);
    HAL_Delay(100);
    BOARD_ReadRawADCDataSync(ad_data);
    SIGNAL_TimeDataQ15 timeData = {
        .timeData = ad_data,
        .offset = 0,
        .stride = 1,
        .points = 4096,
        .range = 1,
        .sampleRate = bases[i],
        .stripDc = FALSE,
    };
    SIGNAL_SpectrumF32 spectrum;
    SIGNAL_TimeQ15ToSpectrumF32(&timeData, &spectrum, &buffer);
    base_coord[i] = spectrum.peakFreq;
    double snr = SIGNAL_SimpleSNR(&spectrum);
    printf("base %lfMHz: %lfMHz\n", bases[i] / 1e6, base_coord[i] / 1e6);
    printf("SNR: %lf\n", snr);
  }
  double result = CGS_DetectCarrierFrequency(4, bases, base_coord);
  printf("result: %lfMHz\n", result / 1e6);
  printf("END\n");
}

void APP_Key0Callback(uint8_t event) {
  if (event == KEYS_EVENT_PRESS) {
    LED_On(1);
    APP_RunFrequencyCounter();
  } else if (event == KEYS_EVENT_RELEASE) {
    LED_Off(1);
  }
}

void APP_Key1Callback(uint8_t event) {
  if (event == KEYS_EVENT_PRESS) {
    LED_On(1);
  } else if (event == KEYS_EVENT_RELEASE) {
    LED_Off(1);
  }
}

void APP_InitKeys() {
  keys_pins.keyCount = 2;
  keys_pins.pins[0].pin = KEY0_Pin;
  keys_pins.pins[0].port = KEY0_GPIO_Port;
  keys_pins.pins[0].callback = APP_Key0Callback;
  keys_pins.pins[1].pin = KEY1_Pin;
  keys_pins.pins[1].port = KEY1_GPIO_Port;
  keys_pins.pins[1].callback = APP_Key1Callback;
  keys_pins.htim = &htim7;
  KEYS_Init(&keys_pins);
}

void APP_PollUartCommands() {
  char data[16];
  int readCount = 0;
  readCount = UART_Read(&com_buf, data, 1, 1);
  if (readCount == 0)
    return;
  if (data[0] == 1) {
    BOARD_ReadRawADCDataSync(ad_data);
    UART_SendString(computer, "\xff\xff\xff\xff");
    UART_SendHex(computer, (uint8_t *)ad_data, 4096 * 2);
  } else if (data[0] == 2) {
    APP_RunFrequencyCounter();
  } else {
    printf("Unknown command: %d\n", data[0]);
  }
}

void APP_Init() {
  computer = &huart1;
  APP_InitKeys();
  BOARD_InitLMX2572();
  BOARD_InitAD9269();
  RetargetInit(computer);
  UART_RxBuffer_Init(&com_buf, computer);
  UART_Open(&com_buf);
  KEYS_Start();
  printf("Hello World!\r\n");
}

void APP_Loop() {
  APP_PollUartCommands();
  KEYS_Poll();
}
#include "app.h"
#include "cgs.h"
#include "keys.h"
#include "led.h"
#include "lmx2572_legacy.h"
#include "retarget.h"
#include "serial.h"
#include "signal.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_flash_ex.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_uart.h"
#include "timers.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

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

#define WAVE_SINE 1
#define WAVE_TRIANGLE 2

double APP_SineThresholdHigh = 920;
double APP_SineThresholdLow = 860;
double APP_TriangleThreshold = 700;
double APP_PhaseOffset = 0;

// sort float array in descending order
int float_greater(const void *a, const void *b) {
  float *fa = (float *)a;
  float *fb = (float *)b;
  return *fb - *fa;
}

float APP_GetSpectrumBackground(SIGNAL_SpectrumF32 *spectrum) {
  // Use cfft data as temp buffer
  for (int i = 0; i < spectrum->points; i++) {
    spectrum->cfftData[i] = spectrum->ampData[i];
  }
  qsort(spectrum->cfftData, spectrum->points, sizeof(float), float_greater);
  double mean = 0;
  for (int i = spectrum->points * 0.2; i < spectrum->points * 0.8; i++) {
    mean += spectrum->cfftData[i];
  }
  mean /= spectrum->points * 0.6;
  return mean;
}

int APP_GetAmp(SIGNAL_SpectrumF32 *spectrum, float background, int idx) {
  return (spectrum->ampData[idx - 1] + spectrum->ampData[idx] +
         spectrum->ampData[idx + 1]) - background * 3;
}

int APP_DetectPeakType(SIGNAL_SpectrumF32 *spectrum, float background,
                       int idx) {
  float base_amp = APP_GetAmp(spectrum, background, idx);
  if (base_amp < APP_TriangleThreshold) {
    return 0;
  }
  if (base_amp > APP_SineThresholdHigh) {
    return WAVE_SINE;
  }
  // check if there is a third harmonic
  if (idx * 3 < spectrum->points) {
    float third_harmonic = APP_GetAmp(spectrum, background, idx * 3);
    if (third_harmonic > background * 15 && third_harmonic > base_amp * 0.08 &&
        third_harmonic < base_amp * 0.2) {
      return WAVE_TRIANGLE;
    }
  }
  // check if there is a third harmonic
  if (idx * 5 < spectrum->points) {
    float fifth_harmonic = APP_GetAmp(spectrum, background, idx * 5);
    if (fifth_harmonic > background * 15 && fifth_harmonic > base_amp * 0.02 &&
        fifth_harmonic < base_amp * 0.1) {
      return WAVE_TRIANGLE;
    }
  }
  if (base_amp > APP_SineThresholdLow) {
    return WAVE_SINE;
  } else {
    return WAVE_TRIANGLE;
  }
}

void APP_RunSignalSeprater(BOOL debug) {
  BOARD_ReadRawADCDataSync(ad_data);
  SIGNAL_FFTBufferF32 buffer = {
      .fftBuffer = fft_buffer,
      .magBuffer = mag_buffer,
  };
  SIGNAL_TimeDataQ15 timeData = {.timeData = ad_data,
                                 .offset = 0,
                                 .stride = 1,
                                 .points = 4096,
                                 .range = 2,
                                 .sampleRate = BOARD_FREQ / 40,
                                 .stripDc = TRUE,
                                 .window = SIGNAL_WINDOW_NONE};
  SIGNAL_SpectrumF32 spectrum;
  SIGNAL_TimeQ15ToSpectrumF32(&timeData, &spectrum, &buffer);
  SIGNAL_PeaksF32 peaks;
  double backgound = APP_GetSpectrumBackground(&spectrum);
  SIGNAL_FindPeaksF32(&spectrum, &peaks, backgound * 20, 3);
  if (debug) {
    printf("background: %lf\n", backgound);
  }
  double freq_a = 0;
  double freq_b = 0;
  int type_a = 0;
  int type_b = 0;
  for (int i = 0; i < peaks.count; i++) {
    if (debug) {
      printf("peak %d: %.2lfkHz, amp %.2f, phase %.2f\n", i,
             peaks.peaks[i].freq / 1e3, peaks.peaks[i].amp,
             peaks.peaks[i].phase);
    }
    int type = APP_DetectPeakType(&spectrum, backgound, peaks.peaks[i].index);
    if (type != 0) {
      if (type_a == 0) {
        freq_a = peaks.peaks[i].freq;
        type_a = type;
      } else {
        freq_b = peaks.peaks[i].freq;
        type_b = type;
        break;
      }
    }
  }
  if (debug) {
    printf("freq_a: %.2lfkHz, type: %d\n", freq_a / 1e3, type_a);
    printf("freq_b: %.2lfkHz, type: %d\n", freq_b / 1e3, type_b);
    printf("END\n");
  }
  BOARD_SetTriggerFrequency(5000);
  if (freq_a != 0) {
    BOARD_SetFrequencyA(freq_a);
  }
  if (freq_b != 0) {
    BOARD_SetFrequencyB(freq_b);
  }
  BOARD_SetOutput(type_a, type_b);
  if (!debug) {
    UART_SendString(computer, "\xff\xff\xff\xff");
    UART_SendHex(computer, (uint8_t *)&type_a, 4);
    UART_SendHex(computer, (uint8_t *)&freq_a, 8);
    UART_SendHex(computer, (uint8_t *)&type_b, 4);
    UART_SendHex(computer, (uint8_t *)&freq_b, 8);
  }
}

void APP_Key0Callback(uint8_t event) {
  if (event == KEYS_EVENT_PRESS) {
    LED_On(1);
    // APP_RunFrequencyCounter();
    APP_PhaseOffset += 5;
    if (APP_PhaseOffset > 360) {
      APP_PhaseOffset -= 360;
    }
    BOARD_SetPhaseA(APP_PhaseOffset);
    BOARD_SetPhaseB(APP_PhaseOffset);
  } else if (event == KEYS_EVENT_RELEASE) {
    LED_Off(1);
  }
}

void APP_Key1Callback(uint8_t event) {
  if (event == KEYS_EVENT_PRESS) {
    LED_On(1);
    APP_RunSignalSeprater(TRUE);
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
  } else if (data[0] == 3) {
    APP_RunSignalSeprater(FALSE);
  } else {
    printf("Unknown command: %d\n", data[0]);
  }
}

void APP_Init() {
  computer = &huart1;
  APP_InitKeys();
  BOARD_InitLMX2572();
  BOARD_InitAD9269();
  BOARD_ResetFPGA();
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
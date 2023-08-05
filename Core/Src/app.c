#include "app.h"
#include "cgs.h"
#include "keys.h"
#include "led.h"
#include "lmx2572_legacy.h"
#include "retarget.h"
#include "screen.h"
#include "serial.h"
#include "signal.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_flash.h"
#include "stm32h7xx_hal_flash_ex.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_uart.h"
#include "timers.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

KEYS_Pins keys_pins;

UART_RxBuffer com_buf, scr_buf;
UART_HandleTypeDef *computer;
UART_HandleTypeDef *screen;
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

#define DEBUG_NONE 0
#define DEBUG_MANUAL 1
#define DEBUG_AUTO 2

double APP_SineThresholdHigh = 920;
double APP_SineThresholdLow = 860;
double APP_TriangleThreshold = 700;
int APP_SamplePoints = 4096;
BOOL APP_EnableAutoPhase = FALSE;
double APP_PhaseA = 0;
double APP_PhaseB = 0;
double APP_PhaseOffset = 0;

void APP_UpdatePhase() {
  if (APP_EnableAutoPhase) {
    BOARD_SetPhaseA(APP_PhaseA);
    BOARD_SetPhaseB(APP_PhaseB);
    SCREEN_PrintText("phase", "追踪");
  } else {
    BOARD_SetPhaseA(0);
    BOARD_SetPhaseB(APP_PhaseOffset);
    SCREEN_PrintText("phase", "%d", (int)(APP_PhaseOffset));
    UART_Printf(screen, "slider.val=%d", (int)(APP_PhaseOffset));
    SCREEN_EndLine();
  }
}

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
          spectrum->ampData[idx + 1]) -
         background * 3;
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

void APP_RunSignalSeprater(int debug) {
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
  SIGNAL_FindPeaksF32(&spectrum, &peaks, 10, 3);
  double backgound = APP_GetSpectrumBackground(&spectrum);
  if (debug == DEBUG_MANUAL) {
    printf("background: %lf\n", backgound);
  }
  double freq_a = 0;
  double freq_b = 0;
  double phase_a = 0;
  double phase_b = 0;
  int type_a = 0;
  int type_b = 0;
  for (int i = 0; i < peaks.count; i++) {
    if (debug == DEBUG_MANUAL) {
      printf("peak %d: %.2lfkHz, amp %.2f, phase %.2f\n", i,
             peaks.peaks[i].freq / 1e3, peaks.peaks[i].amp,
             peaks.peaks[i].phase);
    }
    int type = APP_DetectPeakType(&spectrum, backgound, peaks.peaks[i].index);
    if (type != 0) {
      if (type_a == 0) {
        freq_a = peaks.peaks[i].freq;
        type_a = type;
        phase_a = peaks.peaks[i].phase;
      } else {
        freq_b = peaks.peaks[i].freq;
        type_b = type;
        phase_b = peaks.peaks[i].phase;
        break;
      }
    }
  }
  if (debug == DEBUG_MANUAL) {
    printf("freq_a: %.2lfkHz, type: %d\n", freq_a / 1e3, type_a);
    printf("freq_b: %.2lfkHz, type: %d\n", freq_b / 1e3, type_b);
    printf("phase_a - phase_b: %.2lf\n", SIGNAL_PhaseSub(phase_a, phase_b));
    printf("END\n");
  }
  BOARD_SetTriggerFrequency(5000);
  if (freq_a != 0) {
    BOARD_SetFrequencyA(freq_a);
    APP_PhaseA = SIGNAL_PhaseSub(180, phase_a);
    SCREEN_PrintText("freq_a", "%.1lfkHz", freq_a / 1e3);
    SCREEN_PrintText("type_a", "%s", type_a == WAVE_SINE ? "正弦波" : "三角波");
  } else {
    SCREEN_PrintText("freq_a", "0.0kHz");
    SCREEN_PrintText("type_a", "无信号");
  }
  if (freq_b != 0) {
    BOARD_SetFrequencyB(freq_b);
    APP_PhaseB = SIGNAL_PhaseSub(180, phase_b);
    SCREEN_PrintText("freq_b", "%.1lfkHz", freq_b / 1e3);
    SCREEN_PrintText("type_b", "%s", type_b == WAVE_SINE ? "正弦波" : "三角波");
  } else {
    SCREEN_PrintText("freq_b", "0.0kHz");
    SCREEN_PrintText("type_b", "无信号");
  }
  APP_UpdatePhase();
  BOARD_SetOutput(type_a, type_b);
  if (debug == DEBUG_AUTO) {
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
    printf("phase offset: %lf\n", APP_PhaseOffset);
    APP_UpdatePhase();
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

void APP_PollComputerCommands() {
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

void APP_PollScreenCommands() {
  char data[16];
  int readCount = 0;
  readCount = UART_Read(&scr_buf, data, 1, 1);
  if (readCount == 0)
    return;
  if (data[0] == SCREEN_EVENT_TOUCH) {
    readCount = UART_Read(&scr_buf, data, 6, 1000);
    if (readCount < 3)
      return;
    uint8_t page_id = data[0];
    uint8_t item_id = data[1];
    uint8_t event = data[2];
    if (event != 0x01)
      return;
    switch (item_id) {
    case 1:
      LED_On(1);
      APP_RunSignalSeprater(DEBUG_NONE);
      LED_Off(1);
      break;
    case 3:
      APP_PhaseOffset -= 25;
    case 4:
      APP_PhaseOffset -= 4;
    case 5:
      APP_PhaseOffset -= 1;
      if (APP_PhaseOffset < 0) {
        APP_PhaseOffset += 360;
      }
      APP_EnableAutoPhase = FALSE;
      APP_UpdatePhase();
      break;
    case 8:
      APP_PhaseOffset += 25;
    case 7:
      APP_PhaseOffset += 4;
    case 6:
      APP_PhaseOffset += 1;
      if (APP_PhaseOffset > 360) {
        APP_PhaseOffset -= 360;
      }
      APP_EnableAutoPhase = FALSE;
      APP_UpdatePhase();
      break;
    case 18:
      APP_EnableAutoPhase = TRUE;
      APP_UpdatePhase();
      break;
    }
  } else if (data[0] == SCREEN_EVENT_NUMBER_DATA) {
    readCount = UART_Read(&scr_buf, data, 7, 1000);
    if (readCount < 4)
      return;
    uint32_t newPhase = (*(uint32_t *)(data));
    APP_PhaseOffset = newPhase;
    APP_EnableAutoPhase = FALSE;
    APP_UpdatePhase();
  }
}

void APP_Init() {
  LED_On(2);
  HAL_Delay(2000);
  LED_Off(2);
  computer = &huart1;
  screen = &huart6;
  APP_InitKeys();
  BOARD_InitLMX2572();
  BOARD_InitAD9269();
  BOARD_ResetFPGA();
  BOARD_SetTriggerFrequency(5000);
  SCREEN_Init(screen);
  RetargetInit(computer);
  UART_RxBuffer_Init(&com_buf, computer);
  UART_RxBuffer_Init(&scr_buf, screen);
  UART_Open(&com_buf);
  UART_Open(&scr_buf);
  KEYS_Start();
}

void APP_Loop() {
  APP_PollComputerCommands();
  APP_PollScreenCommands();
  KEYS_Poll();
}
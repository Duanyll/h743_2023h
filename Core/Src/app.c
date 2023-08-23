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

#define MODE_MANUAL 0
#define MODE_CONTINUOUS 1
#define MODE_AUTO 2

double APP_SyncFreq = 5e3;
double APP_SineThresholdHigh = 920;
double APP_SineThresholdLow = 860;
double APP_TriangleThreshold = 700;
int APP_SamplePoints = 4096; // 4096 or 1024
BOOL APP_EnableAutoPhase = FALSE;
int APP_WorkMode = MODE_MANUAL;
double APP_FreqA = 0;
double APP_FreqB = 0;
double APP_PhaseA = 0;
double APP_PhaseB = 0;
double APP_PhaseOffset = 0;

double APP_PhaseOffsetTable[13][3] = {
    {20e3, 40e3, 4},   {20e3, 60e3, 8}, {20e3, 80e3, 11}, {20e3, 100e3, 14},
    {25e3, 50e3, 3.5}, {25e3, 75e3, 6}, {25e3, 100e3, 9}, {30e3, 60e3, 3},
    {30e3, 90e3, 5},   {35e3, 70e3, 3}, {40e3, 80e3, 3},  {45e3, 90e3, 2},
    {50e3, 100e3, 2}};

// double APP_PhaseOffsetTable[13][3] = {
//     {20e3, 40e3, 4},   {20e3, 60e3, 8}, {20e3, 80e3, 11}, {20e3, 100e3, 14},
//     {25e3, 50e3, 3.5}, {25e3, 75e3, 6}, {25e3, 100e3, 9}, {30e3, 60e3, 3},
//     {30e3, 90e3, 5},   {35e3, 70e3, 3}, {40e3, 80e3, 3},  {45e3, 90e3, 2},
//     {50e3, 100e3, 2}};

double APP_LookupPhaseOffset(double freqA, double freqB) {
  // find cloest freq pair
  int min_idx = 0;
  double min_dist = 1e10;
  for (int i = 0; i < 13; i++) {
    double dist = fabs(APP_PhaseOffsetTable[i][0] - freqA) +
                  fabs(APP_PhaseOffsetTable[i][1] - freqB);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return APP_PhaseOffsetTable[min_idx][2];
}


double APP_PhaseOffsetTableA[16][2] = {
  {20e3, 356}, 
  {25e3, 358}, 
  {30e3, 0},
  {35e3, 1.5}, 
  {40e3, 2.5}, 
  {45e3, 3}, 
  {50e3, 5}, 
  {55e3, 5.6}, 
  {60e3, 7}, 
  {65e3, 7}, 
  {70e3, 8}, 
  {75e3, 8.9}, 
  {80e3, 9.8}, 
  {85e3, 10.8}, 
  {90e3, 11.8}, 
  {95e3, 13}
};

double APP_PhaseOffsetTableB[16][2] = {
  {25e3, 358}, 
  {30e3, 359},
  {35e3, 1}, 
  {40e3, 1}, 
  {45e3, 2}, 
  {50e3, 3}, 
  {55e3, 4}, 
  {60e3, 5}, 
  {65e3, 6}, 
  {70e3, 6.5}, 
  {75e3, 8}, 
  {80e3, 9}, 
  {85e3, 10.5}, 
  {90e3, 11}, 
  {95e3, 12}, 
  {100e3, 12.5}
};

double APP_LookupPhaseOffsetA(double freq) {
  int min_idx = 0;
  double min_dist = 1e10;
  for (int i = 0; i < 16; i++) {
    double dist = fabs(APP_PhaseOffsetTableA[i][0] - freq);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return APP_PhaseOffsetTableA[min_idx][1];
}

double APP_LookupPhaseOffsetB(double freq) {
  int min_idx = 0;
  double min_dist = 1e10;
  for (int i = 0; i < 16; i++) {
    double dist = fabs(APP_PhaseOffsetTableB[i][0] - freq);
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }
  return APP_PhaseOffsetTableB[min_idx][1];
}

void APP_UpdatePhase() {
  if (APP_EnableAutoPhase) {
    BOARD_SetPhaseA(
        SIGNAL_PhaseAdd(APP_PhaseA, APP_LookupPhaseOffsetA(APP_FreqA)));
    BOARD_SetPhaseB(
        SIGNAL_PhaseAdd(APP_PhaseB, APP_LookupPhaseOffsetB(APP_FreqB)));
    SCREEN_PrintText("phase", "追踪");
    SCREEN_PrintText("t1", "相位");
  } else {
    BOARD_SetPhaseA(0);
    double totalPhase = SIGNAL_PhaseAdd(
        APP_PhaseOffset, APP_LookupPhaseOffset(APP_FreqA, APP_FreqB));
    BOARD_SetPhaseB(totalPhase);
    SCREEN_PrintText("phase", "%d", (int)(APP_PhaseOffset));
    SCREEN_PrintText("t1", "%.2lfus", APP_PhaseOffset / 360 / APP_FreqB * 1e6);
    UART_Printf(screen, "slider.val=%d", (int)(APP_PhaseOffset));
    SCREEN_EndLine();
  }

  // BOARD_SetPhaseA(SIGNAL_PhaseAdd(APP_PhaseA, APP_PhaseOffset));
  // BOARD_SetPhaseB(SIGNAL_PhaseAdd(APP_PhaseB, APP_PhaseOffset));
  // SCREEN_PrintText("phase", "%d", (int)(APP_PhaseOffset));
  // UART_Printf(screen, "slider.val=%d", (int)(APP_PhaseOffset));
  // SCREEN_EndLine();
}

void APP_UpdateSyncFreq() {
  BOARD_SetTriggerFrequency(APP_SyncFreq);
  SCREEN_PrintText("sync_freq", "%.1lfkHz", APP_SyncFreq / 1e3);
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

float APP_GetAmp(SIGNAL_SpectrumF32 *spectrum, float background, int idx) {
  float res = (spectrum->ampData[idx - 1] + spectrum->ampData[idx] +
               spectrum->ampData[idx + 1]) -
              background * 3;
  if (APP_SamplePoints == 1024) {
    res *= 4;
  }
  return res;
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
                                 .points = APP_SamplePoints,
                                 .range = 2,
                                 .sampleRate = BOARD_FREQ / BOARD_DOWNSAMPLE,
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
    if (freq_a != APP_FreqA) {
      APP_FreqA = freq_a;
      BOARD_SetFrequencyA(freq_a);
    }
    APP_PhaseA = SIGNAL_PhaseSub(180, phase_a);
    SCREEN_PrintText("freq_a", "%.1lfkHz", freq_a / 1e3);
    SCREEN_PrintText("type_a", "%s", type_a == WAVE_SINE ? "正弦波" : "三角波");
  } else {
    SCREEN_PrintText("freq_a", "0.0kHz");
    SCREEN_PrintText("type_a", "无信号");
  }
  if (freq_b != 0) {
    if (freq_b != APP_FreqB) {
      APP_FreqB = freq_b;
      BOARD_SetFrequencyB(freq_b);
    }
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
    APP_RunSignalSeprater(DEBUG_AUTO);
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
    if (page_id == 0x00) {
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
    } else if (page_id == 0x01) {
      switch (item_id) {
      case 3:
        APP_SyncFreq -= 500;
        if (APP_SyncFreq < 0) {
          APP_SyncFreq = 5000;
        }
        APP_UpdateSyncFreq();
        break;
      case 4:
        APP_SyncFreq += 500;
        if (APP_SyncFreq > 5000) {
          APP_SyncFreq = 0;
        }
        APP_UpdateSyncFreq();
        break;
      case 9:
        APP_SamplePoints = 1024;
        SCREEN_PrintText("points", "1024");
        break;
      case 10:
        APP_SamplePoints = 4096;
        SCREEN_PrintText("points", "4096");
        break;
      case 13:
        APP_WorkMode = MODE_MANUAL;
        SCREEN_PrintText("mode", "关");
        break;
      case 14:
        APP_WorkMode = MODE_CONTINUOUS;
        SCREEN_PrintText("mode", "开");
        break;
      case 27:
        APP_WorkMode = MODE_AUTO;
        SCREEN_PrintText("mode", "自动");
        break;
      case 17:
        APP_SineThresholdHigh -= 5;
        SCREEN_PrintText("th1", "%d", (int)APP_SineThresholdHigh);
        break;
      case 18:
        APP_SineThresholdHigh += 5;
        SCREEN_PrintText("th1", "%d", (int)APP_SineThresholdHigh);
        break;
      case 21:
        APP_SineThresholdLow -= 5;
        SCREEN_PrintText("th2", "%d", (int)APP_SineThresholdLow);
        break;
      case 22:
        APP_SineThresholdLow += 5;
        SCREEN_PrintText("th2", "%d", (int)APP_SineThresholdLow);
        break;
      case 25:
        APP_TriangleThreshold -= 5;
        SCREEN_PrintText("th3", "%d", (int)APP_TriangleThreshold);
        break;
      case 26:
        APP_TriangleThreshold += 5;
        SCREEN_PrintText("th3", "%d", (int)APP_TriangleThreshold);
        break;
      }
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
  APP_UpdateSyncFreq();
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
  if (APP_WorkMode == MODE_CONTINUOUS) {
    APP_RunSignalSeprater(DEBUG_NONE);
  }
}
#include "main.h"
#include "keys.h"
#include <stdint.h>

void BOARD_InitAD9269();
void BOARD_InitLMX2572();

#define BOARD_FREQ 40.96e6

void BOARD_ResetFPGA();

#define BOARD_OUTPUT_DISABLE 0
#define BOARD_OUTPUT_SINE 1
#define BOARD_OUTPUT_TRIANGLE 2

void BOARD_ReadRawADCDataSync(int16_t *data);
void BOARD_SetTriggerFrequency(double freq);
void BOARD_SetFrequencyA(double freq);
void BOARD_SetFrequencyB(double freq);
void BOARD_SetOutput(int mode_a, int mode_b);
void BOARD_SetPhaseA(double phase_deg);
void BOARD_SetPhaseB(double phase_deg);
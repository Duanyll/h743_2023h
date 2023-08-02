#include "main.h"
#include "keys.h"
#include <stdint.h>

void BOARD_InitAD9269();
void BOARD_InitLMX2572();

void BOARD_ReadRawADCDataSync(int16_t *data);
#ifndef KEYPAD_H_
#define KEYPAD_H_
#include "stdint.h"
#include "stm32f1xx_hal.h"

#define KEYPAD_ROW 4
#define KEYPAD_COL 4


void Keypad_Handle();
void Keypad_Init();


#endif /* KEYPAD_H_ */

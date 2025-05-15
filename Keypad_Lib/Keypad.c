#include "Keypad.h"

const uint8_t key_code[KEYPAD_ROW][KEYPAD_COL] =
{
		{'1','2','3','A'},
		{'4','5','6','B'},
		{'7','8','9','C'},
		{'*','0','#','D'},
};

uint8_t key_current;
static uint8_t key_last;
static uint8_t key_debounce = 0;
//static uint8_t key_test;
static uint8_t debounecing = 0;
static uint32_t t_debounce;
static uint32_t t_start_press;
static uint8_t is_press;

static GPIO_TypeDef *Keypad_RowPort[KEYPAD_ROW] = {GPIOA, GPIOA, GPIOA, GPIOA};
static uint16_t Keypad_RowPin[KEYPAD_ROW] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3};

static GPIO_TypeDef *Keypad_ColPort[KEYPAD_COL] = {GPIOA, GPIOA, GPIOA, GPIOA};
static uint16_t Keypad_ColPin[KEYPAD_COL] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7};

__weak void KeypadPressingCallback(uint8_t key)
{

}

__weak void KeypadRealeaseCallback(uint8_t key)
{

}

__weak void KeypadPressingTimeoutCallback(uint8_t key)
{

}

__weak void KeypadPressingShortCallback(uint8_t key)
{

}

void KeyPad_Select_Row(uint8_t row)
{
	HAL_GPIO_WritePin(Keypad_RowPort[row], Keypad_RowPin[row], GPIO_PIN_RESET);
}

void KeyPad_UnSelect_Row()
{
	for(uint8_t  row = 0;row < KEYPAD_ROW; row++)
	{
		HAL_GPIO_WritePin(Keypad_RowPort[row], Keypad_RowPin[row], GPIO_PIN_SET);
	}
}

static uint8_t KeyPad_GetKey()
{
	for(uint8_t row = 0; row < KEYPAD_ROW; row++)
	{
		KeyPad_UnSelect_Row();
		KeyPad_Select_Row(row);
		for(uint8_t col = 0; col < KEYPAD_COL; col++)
		{
			if(HAL_GPIO_ReadPin(Keypad_ColPort[col], Keypad_ColPin[col]) == 0)
			{
				return key_code[row][col];
			}
		}
	}
	return 0;
}

static void Keypad_Filter()
{

	uint8_t key = KeyPad_GetKey();
	// Khi van dang nhieu;
	if(key != key_debounce)
	{
		debounecing = 1;
		t_debounce = HAL_GetTick();
		key_debounce = key;
	}
	// Trang thai da xac lap;
	if(debounecing && (HAL_GetTick() - t_debounce) >= 15)
	{
		key_current = key_debounce;
		debounecing = 0;
	}

}

void Keypad_Handle()
{
	Keypad_Filter();
	if(key_current != key_last)
	{
		if(key_current != 0)
		{
			is_press = 1;
			t_start_press = HAL_GetTick();
			KeypadPressingCallback(key_current);
		}
		else
		{
			is_press = 0;
			if(HAL_GetTick() - t_start_press <= 1000)
			{
				KeypadPressingShortCallback(key_last);
			}
			KeypadRealeaseCallback(key_last);
		}
		key_last = key_current;
	}

//	if(is_press && (HAL_GetTick() - t_start_press >= 3000) ) //nhan giu phim
//	{
//		key_test = key_current;
//		KeypadPressingTimeoutCallback(key_current);
//		is_press = 0;
//	}
    if (is_press)
    {
    	HAL_Delay(100);
        KeypadPressingCallback(key_current);
    }
}

void Keypad_Init()
{
	KeyPad_UnSelect_Row();
}
















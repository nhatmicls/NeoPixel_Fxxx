/*
 * neopixel.c
 *
 *  Created on: Dec 12, 2020
 *      Author: cptpr
 */

#include "neopixel.h"

uint16_t RWL[50]={ON,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,ON,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,ON,ON,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF,ON,OFF,OFF,OFF,OFF,OFF,OFF,OFF,OFF};

uint16_t numbers_of_led;
type_led type_of_led=NOTDEFINE;
extern modeled_t modeled=HALT;

uint16_t buffer[48];
uint8_t present_led_counting=0;

extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_tim8_ch1;

void init_neopixel(uint16_t MAX_STRING_LED_LENGTH, type_led in_type_of_led)
{
	if(MAX_STRING_LED_LENGTH<MAX_LED)
		numbers_of_led=MAX_STRING_LED_LENGTH;
	else
		numbers_of_led==MAX_LED;
	type_of_led=in_type_of_led;
}

void render_neopixel()
{
	uint_fast16_t var;
	if(type_of_led==NOTDEFINE)
	{
		present_led_counting=0;
		for (var = 0; var < 8; ++var)
		{
			buffer[var]=RWL[var];
			buffer[var+8]=RWL[var+8];
			buffer[var+16]=RWL[var+16];
			buffer[var+24]=RWL[var+24];
			buffer[var+32]=RWL[var+32];
			buffer[var+40]=RWL[var+40];
		}
		HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, (uint32_t *)buffer, 48);
	}
	else
	{
		__NOP();
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	uint_fast8_t var;
	if(present_led_counting+2<numbers_of_led)
	{
		++present_led_counting;
	}
	else if(present_led_counting<numbers_of_led)
	{
		++present_led_counting;
		for (var = 0; var < 8; ++var) {
			buffer[var+24]=0;
			buffer[var+32]=0;
			buffer[var+40]=0;
		}
	}
	else
	{
		modeled=HALT;
		HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_1);
	}
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
	uint_fast8_t var;
	if(present_led_counting+2<numbers_of_led)
	{
		++present_led_counting;
	}
	else if(present_led_counting<numbers_of_led)
	{
		++present_led_counting;
		for (var = 0; var < 8; ++var) {
			buffer[var]=0;
			buffer[var+8]=0;
			buffer[var+16]=0;
		}
	}
}

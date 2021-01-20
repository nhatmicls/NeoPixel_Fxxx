/*
 * neopixel.c
 *
 *  Created on: Dec 12, 2020
 *      Author: cptpr
 */

#include "neopixel.h"

#define TIMER_CHOICE 3
#define BUFFERLED 10
#define STARTBUFFERLED BUFFERLED-8
#define ENDBUFFERLED BUFFERLED

extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_tim8_ch1;
extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_tim3_ch4;

void all_black_render(void);
void render_neopixel(void);

uint16_t numbers_of_led;
type_led type_of_led=NOTDEFINE;
_mode_t mode=HALT,last_mode=HALT;

uint16_t buffer[50];
uint8_t present_led_counting=0,counting=0;

color allrgb[BUFFERLED];
uint8_t enable_transmit=0;

void init_neopixel(uint16_t MAX_STRING_LED_LENGTH, type_led in_type_of_led)
{
	uint_fast16_t var;
	if(MAX_STRING_LED_LENGTH<MAX_LED)
		numbers_of_led=MAX_STRING_LED_LENGTH;
	else
		numbers_of_led==MAX_LED;
	type_of_led=in_type_of_led;
	all_black_render();
}

void all_black_render(void)
{
	uint_fast16_t var;
	for (var = 0; var < ENDBUFFERLED; ++var)
	{
		allrgb[var].blue=0;
		allrgb[var].red=0;
		allrgb[var].green=0;
	}
	mode=START;
	render_neopixel();
}

void one_color_render(uint8_t blue,uint8_t red,uint8_t green)
{
	uint_fast16_t var;
	for (var = STARTBUFFERLED; var < ENDBUFFERLED; ++var)
	{
		allrgb[var].blue=blue;
		allrgb[var].red=red;
		allrgb[var].green=green;
	}
	mode=START;
	render_neopixel();
}

void render_falling_mode(uint8_t blue,uint8_t red,uint8_t green,uint8_t delay)
{
	uint_fast16_t var,var1;
	for (var = STARTBUFFERLED; var < ENDBUFFERLED; ++var) {
		for (var1 = STARTBUFFERLED; var1 < ENDBUFFERLED; ++var1)
		{
			allrgb[var1].green=green*((var==var1)?1:0);
			allrgb[var1].red=red*((var==var1)?1:0);
			allrgb[var1].blue=blue*((var==var1)?1:0);
		}
		HAL_Delay(delay);
		render_neopixel();
	}
}

void render_upping_mode(uint8_t blue,uint8_t red,uint8_t green,uint8_t delay)
{
	int_fast16_t var,var1;
	for (var = ENDBUFFERLED-1; var >= STARTBUFFERLED; --var) {
		for (var1 = STARTBUFFERLED; var1 < ENDBUFFERLED; ++var1)
		{
			allrgb[var1].green=green*((var==var1)?1:0);
			allrgb[var1].red=red*((var==var1)?1:0);
			allrgb[var1].blue=blue*((var==var1)?1:0);
		}
		HAL_Delay(delay);
		render_neopixel();
	}
}

void render(_mode_t INPUT_MODE,uint8_t blue,uint8_t red,uint8_t green,uint8_t delay)
{
	mode=INPUT_MODE;
	if(last_mode!=mode)
		counting=0;
	switch (INPUT_MODE) {
		case FALLING:
			render_falling_mode(blue, red, green, delay);
			break;
		case UPPING:
			render_upping_mode(blue, red, green, delay);
			break;
		default:
			break;
	}
	last_mode=mode;
	mode=HALT;
}

void render_neopixel()
{
	uint_fast16_t var;
	if(type_of_led!=NOTDEFINE)
	{
		present_led_counting=0;
		for (var = 0; var < 8; ++var)
		{
			buffer[var]=0<<(((allrgb[0].green<<var)&0x80)>0);
			buffer[var+8]=0<<(((allrgb[0].red<<var)&0x80)>0);
			buffer[var+16]=0<<(((allrgb[0].blue<<var)&0x80)>0);
			buffer[var+24]=0<<(((allrgb[1].green<<var)&0x80)>0);
			buffer[var+32]=0<<(((allrgb[1].red<<var)&0x80)>0);
			buffer[var+40]=0<<(((allrgb[1].blue<<var)&0x80)>0);
		}
#if(TIMER_CHOICE==3)
		HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, (uint32_t *)buffer, 48);
#else
		HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, (uint32_t *)buffer, 48);
#endif
	}
	else
	{
		__NOP();
	}
}

void prepare_next_led(uint16_t position,uint8_t alpha)
{
	uint_fast8_t var;
	++position;
	if(alpha==1)
	{
		for (var = 0; var < 8; ++var) {
			buffer[var]=OFF<<(((allrgb[position].green<<var)&0x80)>0);
			buffer[var+8]=OFF<<(((allrgb[position].red<<var)&0x80)>0);
			buffer[var+16]=OFF<<(((allrgb[position].blue<<var)&0x80)>0);
		}
		__NOP();
	}
	else
	{
		for (var = 0; var < 8; ++var) {
			buffer[var+24]=OFF<<(((allrgb[position].green<<var)&0x80)>0);
			buffer[var+32]=OFF<<(((allrgb[position].red<<var)&0x80)>0);
			buffer[var+40]=OFF<<(((allrgb[position].blue<<var)&0x80)>0);
		}
		__NOP();
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	uint_fast8_t var;
	if(present_led_counting+2<numbers_of_led)
	{
		++present_led_counting;
		prepare_next_led(present_led_counting,0);
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
		enable_transmit=0;
		mode=HALT;
#if(TIMER_CHOICE==3)
		HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_1);
#else
		HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_4);
#endif
	}
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
	uint_fast8_t var;
	if(enable_transmit==0)
	{
		if(present_led_counting+2<numbers_of_led)
		{
			++present_led_counting;
			prepare_next_led(present_led_counting,1);
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
	else
		enable_transmit++;
}

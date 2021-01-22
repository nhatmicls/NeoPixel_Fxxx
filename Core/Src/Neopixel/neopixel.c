/*
 * neopixel.c
 *
 *  Created on: Dec 12, 2020
 *      Author: cptpr
 */

#include "neopixel.h"

void all_black_render(void);
void render_neopixel(void);

uint16_t numbers_of_led=BUFFERLED;
type_led type_of_led=NOTDEFINE;
_mode_t mode=HALT;

uint16_t buffer[50];
uint8_t present_led_counting=0;

color allrgb[BUFFERLED];
uint8_t enable_transmit=0;

uint8_t dutyoff=WSOFF;

static uint8_t *hsvtorbg(float H,float S,float V)
{
	static uint8_t HVS[3];
	float C=S*V;
	float A=fmod(H/60.0, 2);
	float X1=(1.0-fabs((float)(A-1.0)));
	float X=C*X1;
	float m=V-C;
	float r=0,b=0,g=0;
	if(H >= 0 && H < 60){
		r = C,g = X,b = 0;
	}
	else if(H >= 60 && H < 120){
		r = X,g = C,b = 0;
	}
	else if(H >= 120 && H < 180){
		r = 0,g = C,b = X;
	}
	else if(H >= 180 && H < 240){
		r = 0,g = X,b = C;
	}
	else if(H >= 240 && H < 300){
		r = X,g = 0,b = C;
	}
	else{
		r = C,g = 0,b = X;
	}
	HVS[0] = (r+m)*255;
	HVS[1] = (g+m)*255;
	HVS[2] = (b+m)*255;
	return HVS;
}

void init_neopixel(type_led in_type_of_led)
{
	type_of_led=in_type_of_led;
	switch (type_of_led) {
		case WS2812:
			dutyoff=WSOFF;
			break;
		case WS2812B:
			dutyoff=WSBOFF;
			break;
		default:
			break;
	}
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
	render_neopixel();
}

void render_rainbow_cycle_mode(uint16_t delay)
{
	uint_fast16_t var,var1;
	static uint16_t angle,angle_cache;
	if(++angle>360)
		angle=0;
	for (var = 0; var < ZONE; ++var) {
		angle_cache=angle+(360/ZONE)*var;
		if(angle_cache>360)
			angle_cache=(angle+(360/ZONE)*var)-360;
		uint8_t *hvs = hsvtorbg(angle_cache, 0.9, 0.9);
		for (var1 = var*LEDPERZONE+2; var1 < (var+1)*LEDPERZONE+2; ++var1) {
			allrgb[var1].green=*hvs;
			allrgb[var1].red=*(hvs+1);
			allrgb[var1].blue=*(hvs+2);
		}
	}
	HAL_Delay(delay);
	render_neopixel();
}

void render_falling_mode(uint8_t blue,uint8_t red,uint8_t green,uint16_t delay)
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

void render_raising_mode(uint8_t blue,uint8_t red,uint8_t green,uint16_t delay)
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

void render_neopixel()
{
	mode=START;
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
		HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_1, (uint32_t *)buffer, 48);
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
			buffer[var]=dutyoff<<(((allrgb[position].green<<var)&0x80)>0);
			buffer[var+8]=dutyoff<<(((allrgb[position].red<<var)&0x80)>0);
			buffer[var+16]=dutyoff<<(((allrgb[position].blue<<var)&0x80)>0);
		}
	}
	else
	{
		for (var = 0; var < 8; ++var) {
			buffer[var+24]=dutyoff<<(((allrgb[position].green<<var)&0x80)>0);
			buffer[var+32]=dutyoff<<(((allrgb[position].red<<var)&0x80)>0);
			buffer[var+40]=dutyoff<<(((allrgb[position].blue<<var)&0x80)>0);
		}
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
		HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_1);
	}
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
	uint_fast8_t var;
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

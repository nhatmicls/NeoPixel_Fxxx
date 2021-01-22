/*
 * neopixel.h
 *
 *  Created on: Dec 12, 2020
 *      Author: cptpr
 */

#ifndef SRC_NEOPIXEL_NEOPIXEL_H_
#define SRC_NEOPIXEL_NEOPIXEL_H_

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/*
 * Configure zone
 * */
#define NUMBEROFLED 60
#define LEDPERZONE 1

/*
 * Configure if you know what to change
 * */
#define ON 		58
#define OFF		29
#define RES		0

#define BUFFERLED NUMBEROFLED+2
#define STARTBUFFERLED 2
#define ENDBUFFERLED BUFFERLED
#define ZONE (NUMBEROFLED/LEDPERZONE)

typedef enum
{
	HALT = 0,
	START,
	FALLING,
	UPPING,
	RAINFALL,
	FULL_RED
}_mode_t;

typedef struct
{
	uint8_t red;
	uint8_t blue;
	uint8_t green;
}color;

typedef enum
{
	NOTDEFINE = 0,
	WS2812,
	WS2812B
}type_led;

extern _mode_t mode;
extern color allrgb[BUFFERLED];

void init_neopixel(uint16_t in_numbers_of_led, type_led in_type_of_led);
void all_black_render(void);
void render_neopixel(void);
void render(_mode_t INPUT_MODE,uint8_t blue,uint8_t red,uint8_t green,uint16_t delay);
void one_color_render(uint8_t blue,uint8_t red,uint8_t green);
void render_falling_mode(uint8_t blue,uint8_t red,uint8_t green,uint16_t delay);
void render_raising_mode(uint8_t blue,uint8_t red,uint8_t green,uint16_t delay);
void render_rainbow_mode(uint16_t delay);
void render_rainbow_cycle_mode(uint16_t delay);

#endif /* SRC_NEOPIXEL_NEOPIXEL_H_ */

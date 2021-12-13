# STM32_NeoPixel

A lightwieght, resource saving driver for WS2812 and WS2812B NeoPixel using STM32 HAL, Time PWM and DMA. Implemented on STM32F407 (
STM32F4DISCOVERY) using TIM8, channel 1 (PC6) and DMA.

![image](https://github.com/nhatmicls/NeoPixel_F4/blob/main/20210122_145620.png)

## Tutorial

### STEP 1: THINGS YOU'LL NEED

- An STM32 Board. I'm using STM32F4DISCOVERY
- An LED strip of NeoPixel WS2812 or WS2812B
- And some jumper wires

Simply connect the 5V STM32F4DISCOVERY to 5V LED, GND connect to GND and DIN connect to PC6 pin.

### STEP 2: LIBRARY

- In "Configure zone"
  - Change this "stm32f4xx_hal.h" to what your STM device.
  - "htim8" change this if you choice different timer.
  - "hdma_tim8_ch1" change this if you choice different timer or channel.
  - NUMBEROFLED is your maximum led of led strip.
  - LEDPERZONE is your led per zone (Only effect if you use rainbow mode)(Zone is number of led with same color).
- In "Configure if you know what to change" is default setting and some effect I already code.

### STEP 3: USE LIBRARY

First, time to configure what type of LED you are using.

```c++
init_neopixel(WS2812);
```

Second, make all led is black.

```c++
all_black_render();
```

Some effect I imported to library.

One color in all led

```c++
void one_color_render(uint8_t blue,uint8_t red,uint8_t green);
```

Falling mode

```c++
void render_falling_mode(uint8_t blue,uint8_t red,uint8_t green,uint16_t delay);
```

Raising mode

```c++
void render_raising_mode(uint8_t blue,uint8_t red,uint8_t green,uint16_t delay);
```

Rainbow mode

```c++
void render_rainbow_cycle_mode(uint16_t delay);
```

I also add function to change HSV to RGB

```c++
static uint8_t *hsvtorbg(float H,float S,float V);
```

With **H** for **Hue**, **S** for **Saturation**, **V** for **Value**. And it will return a pointer which point to array **HVS[3]**.

HSV[0] is GREEN.

HSV[1] is RED.

HSV[2] is BLUE.

### STEP 4: CUSTOMIZE EFFECT

All data led must be store in here

```c++
color allrgb[BUFFERLED];
```

**allrbg[BUFFERLED]** is a struct have 3 variable of RGB for per led

```c++
typedef struct
{
  uint8_t red;
  uint8_t blue;
  uint8_t green;
}color;
```

After you save your data to array so you just use "render_neopixel" for display some effect you add.

```c++
void render_neopixel(void);
```

## Reference
https://www.thevfdcollective.com/blog/stm32-and-sk6812-rgbw-led

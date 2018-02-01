/*
 * Copyright notice from original work
 *
 ** Copyright (c) 2014 Jeremy Garff <jer @ jers.net>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     1.  Redistributions of source code must retain the above copyright notice, this list of
 *         conditions and the following disclaimer.
 *     2.  Redistributions in binary form must reproduce the above copyright notice, this list
 *         of conditions and the following disclaimer in the documentation and/or other materials
 *         provided with the distribution.
 *     3.  Neither the name of the owner nor the names of its contributors may be used to endorse
 *         or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <stdarg.h>
#include <getopt.h>
#include <stdbool.h>
#include <semaphore.h>
#include <carmp3lib.h>

static char VERSION[] = "XX.YY.ZZ";


//#include "clk.h"
//#include "gpio.h"
//#include "dma.h"
//#include "pwm.h"
//#include "version.h"

#include <rpi_ws281x/ws2811.h>

// int sem_init (sem_t *semaphore, int pshared, unsigned int arg);
sem_t semaphore;


#define ARRAY_SIZE(stuff)       (sizeof(stuff) / sizeof(stuff[0]))

#define RED(x) ((x >> 16)&0xff)
#define GREEN(x) ((x >> 8)&0xff)
#define BLUE(x) ((x)&0xff)

#define RGB_VAL(r,g,b) (((r & 0xFF )<< 16) | ((g & 0xFF )<< 8) | (b & 0xFF ))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) < (b) ? (b) : (a))



// defaults for cmdline options
#define TARGET_FREQ             800000
#define GPIO_PIN                18
#define DMA                     10
#define STRIP_TYPE              WS2811_STRIP_GBR

#define WIDTH                   16
#define HEIGHT                  1
#define LED_COUNT               (WIDTH * HEIGHT)
#define IR_PORT 				17

int width = WIDTH;
int height = HEIGHT;
int led_count = LED_COUNT;

enum colours
{
red = 		  0x00020000,  // red
orange = 		    0x00020100,  // orange
yellow = 		    0x00020200,  // yellow
green = 		    0x00000200,  // green
lightblue = 		    0x00000202,  // lightblue
blue = 		    0x00000002,  // blue
purple = 		    0x00010001,  // purple
pink = 		    0x00020001,  // pink
white =  0x00202020,
black =  0x00000000
};

int clear_on_exit = 1;

ws2811_t ledstring =
{
    .freq = TARGET_FREQ,
    .dmanum = DMA,
    .channel =
    {
        [0] =
        {
            .gpionum = GPIO_PIN,
            .count = LED_COUNT,
            .invert = 0,
            .brightness = 255,
            .strip_type = STRIP_TYPE,
        },
        [1] =
        {
            .gpionum = 0,
            .count = 0,
            .invert = 0,
            .brightness = 0,
        },
    },
};

ws2811_led_t *matrix;

static uint8_t running = 1;

static int (*modifier)(ws2811_led_t * , ws2811_led_t *, void *) = NULL;

void matrix_render(ws2811_led_t *used_matrix)
{
    int x, y;

    for (x = 0; x < width; x++)
    {
        for (y = 0; y < height; y++)
        {
            ledstring.channel[0].leds[(y * width) + x] = used_matrix[y * width + x];
        }
    }
}

void matrix_clear(void)
{
    int x, y;

    for (y = 0; y < (height ); y++)
    {
        for (x = 0; x < width; x++)
        {
            matrix[y * width + x] = 0;
        }
    }
}

static void ctrl_c_handler(int signum)
{
	(void)(signum);
    running = 0;
}

static void setup_handlers(void)
{
    struct sigaction sa =
    {
        .sa_handler = ctrl_c_handler,
    };

    sigaction(SIGINT, &sa, NULL);
    sigaction(SIGTERM, &sa, NULL);
}


void parseargs(int argc, char **argv, ws2811_t *ws2811)
{
	int index;
	int c;

	static struct option longopts[] =
	{
		{"help", no_argument, 0, 'h'},
		{"dma", required_argument, 0, 'd'},
		{"gpio", required_argument, 0, 'g'},
		{"invert", no_argument, 0, 'i'},
		{"clear", no_argument, 0, 'c'},
		{"strip", required_argument, 0, 's'},
		{"height", required_argument, 0, 'y'},
		{"width", required_argument, 0, 'x'},
		{"version", no_argument, 0, 'v'},
		{0, 0, 0, 0}
	};

	while (1)
	{

		index = 0;
		c = getopt_long(argc, argv, "cd:g:his:vx:y:", longopts, &index);

		if (c == -1)
			break;

		switch (c)
		{
		case 0:
			/* handle flag options (array's 3rd field non-0) */
			break;

		case 'h':
			fprintf(stderr, "%s version %s\n", argv[0], VERSION);
			fprintf(stderr, "Usage: %s \n"
				"-h (--help)    - this information\n"
				"-s (--strip)   - strip type - rgb, grb, gbr, rgbw\n"
				"-x (--width)   - matrix width (default 8)\n"
				"-y (--height)  - matrix height (default 8)\n"
				"-d (--dma)     - dma channel to use (default 5)\n"
				"-g (--gpio)    - GPIO to use\n"
				"                 If omitted, default is 18 (PWM0)\n"
				"-i (--invert)  - invert pin output (pulse LOW)\n"
				"-c (--clear)   - clear matrix on exit.\n"
				"-v (--version) - version information\n"
				, argv[0]);
			exit(-1);

		case 'D':
			break;

		case 'g':
			if (optarg) {
				int gpio = atoi(optarg);
/*
	PWM0, which can be set to use GPIOs 12, 18, 40, and 52.
	Only 12 (pin 32) and 18 (pin 12) are available on the B+/2B/3B
	PWM1 which can be set to use GPIOs 13, 19, 41, 45 and 53.
	Only 13 is available on the B+/2B/PiZero/3B, on pin 33
	PCM_DOUT, which can be set to use GPIOs 21 and 31.
	Only 21 is available on the B+/2B/PiZero/3B, on pin 40.
	SPI0-MOSI is available on GPIOs 10 and 38.
	Only GPIO 10 is available on all models.

	The library checks if the specified gpio is available
	on the specific model (from model B rev 1 till 3B)

*/
				ws2811->channel[0].gpionum = gpio;
			}
			break;

		case 'i':
			ws2811->channel[0].invert=1;
			break;

		case 'c':
			clear_on_exit=1;
			break;

		case 'd':
			if (optarg) {
				int dma = atoi(optarg);
				if (dma < 14) {
					ws2811->dmanum = dma;
				} else {
					printf ("invalid dma %d\n", dma);
					exit (-1);
				}
			}
			break;

		case 'y':
			if (optarg) {
				height = atoi(optarg);
				if (height > 0) {
					ws2811->channel[0].count = height * width;
				} else {
					printf ("invalid height %d\n", height);
					exit (-1);
				}
			}
			break;

		case 'x':
			if (optarg) {
				width = atoi(optarg);
				if (width > 0) {
					ws2811->channel[0].count = height * width;
				} else {
					printf ("invalid width %d\n", width);
					exit (-1);
				}
			}
			break;

		case 's':
			if (optarg) {
				if (!strncasecmp("rgb", optarg, 4)) {
					ws2811->channel[0].strip_type = WS2811_STRIP_RGB;
				}
				else if (!strncasecmp("rbg", optarg, 4)) {
					ws2811->channel[0].strip_type = WS2811_STRIP_RBG;
				}
				else if (!strncasecmp("grb", optarg, 4)) {
					ws2811->channel[0].strip_type = WS2811_STRIP_GRB;
				}
				else if (!strncasecmp("gbr", optarg, 4)) {
					ws2811->channel[0].strip_type = WS2811_STRIP_GBR;
				}
				else if (!strncasecmp("brg", optarg, 4)) {
					ws2811->channel[0].strip_type = WS2811_STRIP_BRG;
				}
				else if (!strncasecmp("bgr", optarg, 4)) {
					ws2811->channel[0].strip_type = WS2811_STRIP_BGR;
				}
				else if (!strncasecmp("rgbw", optarg, 4)) {
					ws2811->channel[0].strip_type = SK6812_STRIP_RGBW;
				}
				else if (!strncasecmp("grbw", optarg, 4)) {
					ws2811->channel[0].strip_type = SK6812_STRIP_GRBW;
				}
				else {
					printf ("invalid strip %s\n", optarg);
					exit (-1);
				}
			}
			break;

		case 'v':
			fprintf(stderr, "%s version %s\n", argv[0], VERSION);
			exit(-1);

		case '?':
			/* getopt_long already reported error? */
			exit(-1);

		default:
			exit(-1);
		}
	}
}

void addColour(int r, int g, int b)
{
int x;
int y;
int red;
int blue;
int green;


	 for (x = 0; x < width; x++)
	 {
		 for (y = 0; y < height; y++)
		 {
			 red = RED(matrix[y * width + x]);
			 green = GREEN(matrix[y * width + x]);
			 blue = BLUE(matrix[y * width + x]);

			 red = MIN(MAX(red + r,0),255);
			 blue = MIN(MAX(blue + b,0),255);
			 green = MIN(MAX(green + g,0),255);

			 matrix[y * width + x] = RGB_VAL(red, green, blue);
		 }
	 }
}

void sineWave(int num)
{
int x;
int y;
int i = 0;
double val;

	 for (x = 0; x < width; x++)
	 {
		 for (y = 0; y < height; y++)
		 {
			 val = (sin(i*2*3.1415/num)+1)/2;
			 matrix[y * width + x] = RGB_VAL(MAX(MIN((int)(RED(matrix[y * width + x])*val),255),0),MAX(MIN((int)(GREEN(matrix[y * width + x])*val),255),0),MAX(MIN((int)(BLUE(matrix[y * width + x])*val),255),0));
			 i++;
		 }
	 }
}

int modulo(int x,int N){
    return (x % N + N) %N;
}

int rotateStart = 0;
int rotateNum = 4;
int rotateDirection = 1;

int rotate(ws2811_led_t * in, ws2811_led_t * out, void *v)
{
	int x;
	int y;
	int i = 0;
	double val;
	int modx;

	for (x = 0; x < width; x++)
	{
		modx = modulo(x + rotateStart/rotateNum  , width);

		for (y = 0; y < height; y++)
		{
			out[y * width + x] = in[y * width + modx];
		}
	}

	rotateStart += rotateDirection;

	return 0;
}

int throbStart = 0;
int throbVal  = 16;


int throb(ws2811_led_t * in, ws2811_led_t * out, void *v)
{
	int x;
	int y;
	int i = 0;
	double val;
	double mult;

	mult = sin(throbStart*2*3.1415/throbVal)+1;

	for (x = 0; x < width; x++)
	{
		for (y = 0; y < height; y++)
		{
			out[y * width + x] = (int)(mult * in[y * width + x]);
		}
	}

	throbStart += 1;

	return 0;
}

void startThrob(int time)
{
modifier = throb;
throbVal  = time;
}



void startRotate(int clockwise)
{
modifier = rotate;
rotateDirection = clockwise ? 1:-1;
}



 void IrReceive(int address, int value, uint32_t tick, bool isRepeat, void * userData)
 {
	 int x;
	 int y;
	 int step = 2;

	 sem_wait(&semaphore);

	 switch (value)
	 {
	 // column 1
	 case 0x00ba45:
		 if (!isRepeat)
		 {
		 for (x = 0; x < width; x++)
		 {
			 for (y = 0; y < height; y++)
			 {
				 matrix[y * width + x] = white;
			 }
		 }
		 }
		 break;

	 case 0x00bb44:
		 addColour(step,0,0);
		 break;


	 case 0x00f807:
		 addColour(-step,0,0);
		 break;

	 case 0x00e916:
		 if (!isRepeat)
		 {
			 sineWave(4);
		 }
		 break;

	 case 0x00f30c:
		 if (!isRepeat)
		 {
			 startThrob(16);
		 }

	 case 0x00f708:
	 case 0x00bd42:
		 break;

		// column2

	 case 0x00b946:
		 if (!isRepeat)
		 {
		 for (x = 0; x < width; x++)
		 {
			 for (y = 0; y < height; y++)
			 {
				 matrix[y * width + x] = black;
			 }
		 }
		 }
		 break;

	 case 0x00bf40:
		 addColour(0,step,0);
		 break;

	 case 0x00ea15:
		 addColour(0,-step,0);
		 break;

	 case 0x00e619:
		 startRotate(1==1);
		 break;
	 case 0x00e718:
	 case 0x00e31c:
	 case 0x00ad52:
		 break;

		 // column 3
	 case 0x00b847:
		 break;

	 case 0x00bc43:
		 addColour(0,0,step);
		 break;

	 case 0x00f609:
		 addColour(0,0,-step);
		 break;

	 case 0x00f20d:
		 startRotate(1==0);
		 break;

	 case 0x00a15e:
	 case 0x00a55a:
	 case 0x00b54a:
		 break;

	 }

 	sem_post (&semaphore);

 }



int main(int argc, char *argv[])
{
    ws2811_return_t ret;
    int i = 0;
    int irPort = IR_PORT;
    ws2811_led_t *modifiedMatrix;

    sprintf(VERSION, "%d.%d.%d", 0, 1, 0);

    parseargs(argc, argv, &ledstring);

    matrix = malloc(sizeof(ws2811_led_t) * width * height);
    modifiedMatrix = malloc(sizeof(ws2811_led_t) * width * height);

    matrix_clear();

    setup_handlers();

    initialise_ir_receiver(irPort, IrReceive, NULL, NULL);

    sem_init(&semaphore, 0, 0);

    if ((ret = ws2811_init(&ledstring)) != WS2811_SUCCESS)
    {
        fprintf(stderr, "ws2811_init failed: %s\n", ws2811_get_return_t_str(ret));
        return ret;
    }

    while (running)
    {

    	sem_wait(&semaphore);

    	if (modifier == NULL)
    	{
    		matrix_render(matrix);
    	}
    	else
    	{
    		modifier(matrix, modifiedMatrix,NULL);
    		matrix_render(modifiedMatrix);
    	}

        if ((ret = ws2811_render(&ledstring)) != WS2811_SUCCESS)
        {
            fprintf(stderr, "ws2811_render failed: %s\n", ws2811_get_return_t_str(ret));
            break;
        }
    	sem_post (&semaphore);

        // 15 frames /sec
        usleep(1000000 / 15);
    }

    if (clear_on_exit)
    {
	matrix_clear();
	matrix_render(matrix);
	ws2811_render(&ledstring);
    }

    ws2811_fini(&ledstring);

    printf ("\n");
    return ret;
}





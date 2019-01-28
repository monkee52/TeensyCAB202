#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <usb_serial.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <graphics.h>
#include <sprite.h>

typedef char* STR;

#ifndef NULL
#define NULL (void*)0
#endif

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
#endif

#pragma region Maths

double min(double a, double b) {
	if (a < b) {
		return a;
	}

	return b;
}

double max(double a, double b) {
	if (a > b) {
		return a;
	}

	return b;
}

double cap(double x, double low, double high) {
	return max(low, min(high, x));
}

#ifndef ABS
#define ABS(x) ((x >= 0) ? x : -x)
#endif

#ifndef SIGN
#define SIGN(x) ((x > 0) - (x < 0))
#endif

#pragma endregion

#pragma region Graphics

// Store this in PROGMEM so that we don't waste RAM on a large static variable
static const unsigned char ASCII2[][5] PROGMEM =
{
	{ 0x00, 0x00, 0x00, 0x00, 0x00 } // 20
	,{ 0x00, 0x00, 0x5f, 0x00, 0x00 } // 21 !
	,{ 0x00, 0x07, 0x00, 0x07, 0x00 } // 22 "
	,{ 0x14, 0x7f, 0x14, 0x7f, 0x14 } // 23 #
	,{ 0x24, 0x2a, 0x7f, 0x2a, 0x12 } // 24 $
	,{ 0x23, 0x13, 0x08, 0x64, 0x62 } // 25 %
	,{ 0x36, 0x49, 0x55, 0x22, 0x50 } // 26 &
	,{ 0x00, 0x05, 0x03, 0x00, 0x00 } // 27 '
	,{ 0x00, 0x1c, 0x22, 0x41, 0x00 } // 28 (
	,{ 0x00, 0x41, 0x22, 0x1c, 0x00 } // 29 )
	,{ 0x14, 0x08, 0x3e, 0x08, 0x14 } // 2a *
	,{ 0x08, 0x08, 0x3e, 0x08, 0x08 } // 2b +
	,{ 0x00, 0x50, 0x30, 0x00, 0x00 } // 2c ,
	,{ 0x08, 0x08, 0x08, 0x08, 0x08 } // 2d -
	,{ 0x00, 0x60, 0x60, 0x00, 0x00 } // 2e .
	,{ 0x20, 0x10, 0x08, 0x04, 0x02 } // 2f /
	,{ 0x3e, 0x51, 0x49, 0x45, 0x3e } // 30 0
	,{ 0x00, 0x42, 0x7f, 0x40, 0x00 } // 31 1
	,{ 0x42, 0x61, 0x51, 0x49, 0x46 } // 32 2
	,{ 0x21, 0x41, 0x45, 0x4b, 0x31 } // 33 3
	,{ 0x18, 0x14, 0x12, 0x7f, 0x10 } // 34 4
	,{ 0x27, 0x45, 0x45, 0x45, 0x39 } // 35 5
	,{ 0x3c, 0x4a, 0x49, 0x49, 0x30 } // 36 6
	,{ 0x01, 0x71, 0x09, 0x05, 0x03 } // 37 7
	,{ 0x36, 0x49, 0x49, 0x49, 0x36 } // 38 8
	,{ 0x06, 0x49, 0x49, 0x29, 0x1e } // 39 9
	,{ 0x00, 0x36, 0x36, 0x00, 0x00 } // 3a :
	,{ 0x00, 0x56, 0x36, 0x00, 0x00 } // 3b ;
	,{ 0x08, 0x14, 0x22, 0x41, 0x00 } // 3c <
	,{ 0x14, 0x14, 0x14, 0x14, 0x14 } // 3d =
	,{ 0x00, 0x41, 0x22, 0x14, 0x08 } // 3e >
	,{ 0x02, 0x01, 0x51, 0x09, 0x06 } // 3f ?
	,{ 0x32, 0x49, 0x79, 0x41, 0x3e } // 40 @
	,{ 0x7e, 0x11, 0x11, 0x11, 0x7e } // 41 A
	,{ 0x7f, 0x49, 0x49, 0x49, 0x36 } // 42 B
	,{ 0x3e, 0x41, 0x41, 0x41, 0x22 } // 43 C
	,{ 0x7f, 0x41, 0x41, 0x22, 0x1c } // 44 D
	,{ 0x7f, 0x49, 0x49, 0x49, 0x41 } // 45 E
	,{ 0x7f, 0x09, 0x09, 0x09, 0x01 } // 46 F
	,{ 0x3e, 0x41, 0x49, 0x49, 0x7a } // 47 G
	,{ 0x7f, 0x08, 0x08, 0x08, 0x7f } // 48 H
	,{ 0x00, 0x41, 0x7f, 0x41, 0x00 } // 49 I
	,{ 0x20, 0x40, 0x41, 0x3f, 0x01 } // 4a J
	,{ 0x7f, 0x08, 0x14, 0x22, 0x41 } // 4b K
	,{ 0x7f, 0x40, 0x40, 0x40, 0x40 } // 4c L
	,{ 0x7f, 0x02, 0x0c, 0x02, 0x7f } // 4d M
	,{ 0x7f, 0x04, 0x08, 0x10, 0x7f } // 4e N
	,{ 0x3e, 0x41, 0x41, 0x41, 0x3e } // 4f O
	,{ 0x7f, 0x09, 0x09, 0x09, 0x06 } // 50 P
	,{ 0x3e, 0x41, 0x51, 0x21, 0x5e } // 51 Q
	,{ 0x7f, 0x09, 0x19, 0x29, 0x46 } // 52 R
	,{ 0x46, 0x49, 0x49, 0x49, 0x31 } // 53 S
	,{ 0x01, 0x01, 0x7f, 0x01, 0x01 } // 54 T
	,{ 0x3f, 0x40, 0x40, 0x40, 0x3f } // 55 U
	,{ 0x1f, 0x20, 0x40, 0x20, 0x1f } // 56 V
	,{ 0x3f, 0x40, 0x38, 0x40, 0x3f } // 57 W
	,{ 0x63, 0x14, 0x08, 0x14, 0x63 } // 58 X
	,{ 0x07, 0x08, 0x70, 0x08, 0x07 } // 59 Y
	,{ 0x61, 0x51, 0x49, 0x45, 0x43 } // 5a Z
	,{ 0x00, 0x7f, 0x41, 0x41, 0x00 } // 5b [
	,{ 0x02, 0x04, 0x08, 0x10, 0x20 } // 5c �
	,{ 0x00, 0x41, 0x41, 0x7f, 0x00 } // 5d ]
	,{ 0x04, 0x02, 0x01, 0x02, 0x04 } // 5e ^
	,{ 0x40, 0x40, 0x40, 0x40, 0x40 } // 5f _
	,{ 0x00, 0x01, 0x02, 0x04, 0x00 } // 60 `
	,{ 0x20, 0x54, 0x54, 0x54, 0x78 } // 61 a
	,{ 0x7f, 0x48, 0x44, 0x44, 0x38 } // 62 b
	,{ 0x38, 0x44, 0x44, 0x44, 0x20 } // 63 c
	,{ 0x38, 0x44, 0x44, 0x48, 0x7f } // 64 d
	,{ 0x38, 0x54, 0x54, 0x54, 0x18 } // 65 e
	,{ 0x08, 0x7e, 0x09, 0x01, 0x02 } // 66 f
	,{ 0x18, 0xa4, 0xa4, 0xa4, 0x7c } // 67 g
	,{ 0x7f, 0x08, 0x04, 0x04, 0x78 } // 68 h
	,{ 0x00, 0x44, 0x7d, 0x40, 0x00 } // 69 i
	,{ 0x20, 0x40, 0x44, 0x3d, 0x00 } // 6a j
	,{ 0x7f, 0x10, 0x28, 0x44, 0x00 } // 6b k
	,{ 0x00, 0x41, 0x7f, 0x40, 0x00 } // 6c l
	,{ 0x7c, 0x04, 0x18, 0x04, 0x78 } // 6d m
	,{ 0x7c, 0x08, 0x04, 0x04, 0x78 } // 6e n
	,{ 0x38, 0x44, 0x44, 0x44, 0x38 } // 6f o
	,{ 0x7c, 0x14, 0x14, 0x14, 0x08 } // 70 p
	,{ 0x08, 0x14, 0x14, 0x18, 0x7c } // 71 q
	,{ 0x7c, 0x08, 0x04, 0x04, 0x08 } // 72 r
	,{ 0x48, 0x54, 0x54, 0x54, 0x20 } // 73 s
	,{ 0x04, 0x3f, 0x44, 0x40, 0x20 } // 74 t
	,{ 0x3c, 0x40, 0x40, 0x20, 0x7c } // 75 u
	,{ 0x1c, 0x20, 0x40, 0x20, 0x1c } // 76 v
	,{ 0x3c, 0x40, 0x30, 0x40, 0x3c } // 77 w
	,{ 0x44, 0x28, 0x10, 0x28, 0x44 } // 78 x
	,{ 0x0c, 0x50, 0x50, 0x50, 0x3c } // 79 y
	,{ 0x44, 0x64, 0x54, 0x4c, 0x44 } // 7a z
	,{ 0x00, 0x08, 0x36, 0x41, 0x00 } // 7b {
	,{ 0x00, 0x00, 0x7f, 0x00, 0x00 } // 7c |
	,{ 0x00, 0x41, 0x36, 0x08, 0x00 } // 7d }
	,{ 0x10, 0x08, 0x08, 0x10, 0x08 } // 7e ?
	,{ 0x78, 0x46, 0x41, 0x46, 0x78 } // 7f ?
};

void scrSetPixel(int8_t x, int8_t y, bool value) {
	if (x >= LCD_X || x < 0 || y >= LCD_Y || y < 0) {
		return;
	}

	set_pixel(x, y, value);
}

void scrDrawLine(int8_t x0, int8_t y0, int8_t x1, int8_t y1, bool value) {
	if (x0 == x1) {
		// Draw vertical line
		for (int i = y0; (y1 > y0) ? i <= y1 : i >= y1; (y1 > y0) ? i++ : i--) {
			scrSetPixel(x0, i, value);
		}
	} else if (y0 == y1) {
		// Draw horizontal line
		for (int i = x0; (x1 > x0) ? i <= x1 : i >= x1; (x1 > x0) ? i++ : i--) {
			scrSetPixel(i, y0, value);
		}
	} else {
		// Get Bresenhaming...
		float dx = x1 - x0;
		float dy = y1 - y0;
		float err = 0.0;
		float derr = ABS(dy / dx);

		for (int x = x0, y = y0; (dx > 0) ? x <= x1 : x >= x1; (dx > 0) ? x++ : x--) {
			scrSetPixel(x, y, value);
			err += derr;

			while (err >= 0.5 && ((dy > 0) ? y <= y1 : y >= y1)) {
				scrSetPixel(x, y, value);
				y += (dy > 0) - (dy < 0);
				err -= 1.0;
			}
		}
	}
}

void scrDrawChar(int8_t x, int8_t y, int8_t c, bool inverse) {
	int char_width = 5;
	int char_height = 8;

	// loop through each pixel in the character array and plot each one individually
	for (unsigned char i = 0; i < char_width; i++) {
		for (unsigned char j = 0; j < char_height; j++) {
			int8_t value = ((pgm_read_byte(&(ASCII2[c - 0x20][i])) & (1 << j)) >> j);

			if (inverse) {
				value = 1 - value;
			}

			scrSetPixel(x + i, y + j, value);
		}
	}
}

void scrDrawString(int8_t x, int8_t y, STR str, bool inverse) {
	unsigned char i = 0;

	// Draw each character until the null terminator is reached
	while (*str != 0) {
		scrDrawChar(x + i * 5, y, *(str), inverse);

		// Add a column of spaces here if you want to space out the lettering.
		// (see lcd.c for a hint on how to do this)
		str++;
		i++;
	}
}

void scrDrawRect(int8_t x0, int8_t y0, int8_t x1, int8_t y1, bool hollow, bool value) {
	int8_t width = abs(x1 - x0);
	int8_t height = abs(y1 - y0);

	x0 = min(x0, x1);
	x1 = x0 + width;
	y0 = min(y0, y1);
	y1 = y0 + height;

	if (hollow) {
		scrDrawLine(x0, y0, x1, y0, value);
		scrDrawLine(x1, y0, x1, y1, value);
		scrDrawLine(x1, y1, x0, y1, value);
		scrDrawLine(x0, y1, x0, y0, value);
	} else {
		for (int8_t i = y0; i < y1; i++) {
			scrDrawLine(x0, i, x1, i, value);
		}
	}
}

#pragma endregion

#pragma region Graphics2

struct tagPoint {
	double x;
	double y;
};

typedef struct tagPoint Point;

struct tagPolygon {
	int length;
	Point* points;
};

typedef struct tagPolygon Polygon, *PPolygon;

struct tagBBox {
	double x;
	double y;
	double width;
	double height;
};

typedef struct tagBBox BBox;

Point createPoint(double x, double y) {
	Point point;

	point.x = x;
	point.y = y;

	return point;
}

PPolygon createPolygon(int length) {
	++length;

	PPolygon polygon = (PPolygon)malloc(sizeof(Polygon));

	polygon->length = length;
	polygon->points = (Point*)malloc(sizeof(Point) * length);

	return polygon;
}

void translatePolygon(PPolygon polygon, Point amt) {
	for (int i = 0; i < polygon->length; i++) {
		polygon->points[i].x += amt.x;
		polygon->points[i].y += amt.y;
	}
}

void rotatePolygon(PPolygon polygon, double amt) {
	for (int i = 0; i < polygon->length; i++) {
		double x = (double)cos(amt) * polygon->points[i].x - (double)sin(amt) * polygon->points[i].y;
		double y = (double)sin(amt) * polygon->points[i].x + (double)cos(amt) * polygon->points[i].y;

		polygon->points[i].x = x;
		polygon->points[i].y = y;
	}
}

BBox getPolygonBBox(PPolygon polygon) {
	BBox bbox = { 0 };

	if (polygon->length > 0) {
		double minX = polygon->points[0].x;
		double minY = polygon->points[0].y;
		double maxX = polygon->points[0].x;
		double maxY = polygon->points[0].y;

		for (int i = 1; i < polygon->length; i++) {
			if (polygon->points[i].x < minX) {
				minX = polygon->points[i].x;
			}

			if (polygon->points[i].y < minY) {
				minY = polygon->points[i].y;
			}

			if (polygon->points[i].x > maxX) {
				maxX = polygon->points[i].x;
			}

			if (polygon->points[i].y > maxY) {
				maxY = polygon->points[i].y;
			}
		}

		bbox.x = minX;
		bbox.y = minY;
		bbox.width = maxX - minX;
		bbox.height = maxY - minY;
	}

	return bbox;
}

void destroyPolygon(PPolygon polygon) {
	if (polygon != NULL) {
		if (polygon->points != NULL) {
			free(polygon->points);

			polygon->points = NULL;
		}

		free(polygon);
	}
}

void drawPolygon(PPolygon polygon) {
	for (int i = 0; i < polygon->length; i++) {
		int j = (i + 1) % polygon->length;

		scrDrawLine(polygon->points[i].x, polygon->points[i].y, polygon->points[j].x, polygon->points[j].y, true);
	}
}

void circularisePolygon(PPolygon polygon, double r) {
	for (int i = 0; i < polygon->length; i++) {
		double t = i / ((double)polygon->length - 1.0f) * 2.0f * M_PI;

		polygon->points[i].x = r * (double)cos(t);
		polygon->points[i].y = r * (double)sin(t);
	}

	BBox bbox = getPolygonBBox(polygon);

	translatePolygon(polygon, createPoint(bbox.width / 2.0f, bbox.height / 2.0f));
}

#pragma endregion

#pragma region IO

enum tagLed {
	LedLeft,
	LedRight
};

typedef enum tagLed Led;

enum tagButton {
	ButtonLeft,
	ButtonRight
};

typedef enum tagButton Button;

enum tagJoystick {
	JoystickNone,
	JoystickUp,
	JoystickRight,
	JoystickDown,
	JoystickLeft,
	JoystickCenter
};

typedef enum tagJoystick Joystick;

enum tagTrimpot {
	TrimpotLeft,
	TrimpotRight
};

typedef enum tagTrimpot Trimpot;

enum tagContrast {
	ContrastLow,
	ContrastDefault,
	ContrastHigh,
	Contrast_length
};

typedef enum tagContrast Contrast;

#define BTN_DPAD_LEFT 0
#define BTN_DPAD_RIGHT 1
#define BTN_DPAD_UP 2
#define BTN_DPAD_DOWN 3
#define BTN_DPAD_CENTER 4
#define BTN_LEFT 5
#define BTN_RIGHT 6

volatile unsigned long overflow_count;
volatile unsigned char btn_hists[7];
volatile bool btn_states[7];


ISR(TIMER0_OVF_vect) {
	overflow_count++;

	btn_hists[BTN_LEFT] <<= 1;
	btn_hists[BTN_LEFT] |= (PINF & (1 << PF6)) >> PF6;

	if (btn_hists[BTN_LEFT] == 0xff) {
		btn_states[BTN_LEFT] = true;
	} else if (btn_hists[BTN_LEFT] == 0x00) {
		btn_states[BTN_LEFT] = false;
	}

	btn_hists[BTN_RIGHT] <<= 1;
	btn_hists[BTN_RIGHT] |= (PINF & (1 << PF5)) >> PF5;

	if (btn_hists[BTN_RIGHT] == 0xff) {
		btn_states[BTN_RIGHT] = true;
	} else if (btn_hists[BTN_RIGHT] == 0x00) {
		btn_states[BTN_RIGHT] = false;
	}

	btn_hists[BTN_DPAD_LEFT] <<= 1;
	btn_hists[BTN_DPAD_LEFT] |= (PINB & (1 << PB1)) >> PB1;

	if (btn_hists[BTN_DPAD_LEFT] == 0xff) {
		btn_states[BTN_DPAD_LEFT] = true;
	} else if (btn_hists[BTN_DPAD_LEFT] == 0x00) {
		btn_states[BTN_DPAD_LEFT] = false;
	}

	btn_hists[BTN_DPAD_RIGHT] <<= 1;
	btn_hists[BTN_DPAD_RIGHT] |= (PIND & (1 << PD0)) >> PD0;

	if (btn_hists[BTN_DPAD_RIGHT] == 0xff) {
		btn_states[BTN_DPAD_RIGHT] = true;
	} else if (btn_hists[BTN_DPAD_RIGHT] == 0x00) {
		btn_states[BTN_DPAD_RIGHT] = false;
	}

	btn_hists[BTN_DPAD_UP] <<= 1;
	btn_hists[BTN_DPAD_UP] |= (PIND & (1 << PD1)) >> PD1;

	if (btn_hists[BTN_DPAD_UP] == 0xff) {
		btn_states[BTN_DPAD_UP] = true;
	} else if (btn_hists[BTN_DPAD_UP] == 0x00) {
		btn_states[BTN_DPAD_UP] = false;
	}

	btn_hists[BTN_DPAD_DOWN] <<= 1;
	btn_hists[BTN_DPAD_DOWN] |= (PINB & (1 << PB7)) >> PB7;

	if (btn_hists[BTN_DPAD_DOWN] == 0xff) {
		btn_states[BTN_DPAD_DOWN] = true;
	} else if (btn_hists[BTN_DPAD_DOWN] == 0x00) {
		btn_states[BTN_DPAD_DOWN] = false;
	}

	btn_hists[BTN_DPAD_CENTER] <<= 1;
	btn_hists[BTN_DPAD_CENTER] |= (PINB & (1 << PB0)) >> PB0;

	if (btn_hists[BTN_DPAD_CENTER] == 0xff) {
		btn_states[BTN_DPAD_CENTER] = true;
	} else if (btn_hists[BTN_DPAD_CENTER] == 0x00) {
		btn_states[BTN_DPAD_CENTER] = false;
	}
}

static inline void setup() {
	// Set clock speed to 8 MHz
	CLKPR = 0x80;
	CLKPR = 0x01;

	DDRF = 0b00000000;

	DDRB = (1 << PB2) | (1 << PB3);
	PORTB = 0b00000000;

	// ADC
	// Prescale /64
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);

	// Timer0
	// Normal timer mode
	TCCR0B &= ~(1 << WGM02);

	// Prescale /8
	TCCR0B &= ~7;
	TCCR0B |= 2;

	// Interrupt on overflow
	TIMSK0 |= (1 << TOIE0);

	// Enable global interrupts
	sei();

	usb_init();

	_delay_ms(20);

	lcd_init(LCD_DEFAULT_CONTRAST);

	clear_screen();
}

uint32_t mstime() {
	unsigned long counts;

	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		counts = overflow_count;
	}

	return (double)counts * .255;
}

void setLed(Led led, bool on) {
	if (led == LedLeft) {
		if (on) {
			PORTB |= (1 << PB2);
		} else {
			PORTB &= ~(1 << PB2);
		}
	} else if (led == LedRight) {
		if (on) {
			PORTB |= (1 << PB3);
		} else {
			PORTB &= ~(1 << PB3);
		}
	}
}

bool getButton(Button sw) {
	if (sw == ButtonLeft) {
		return btn_states[BTN_LEFT];
	} else {
		return btn_states[BTN_RIGHT];
	}
}

bool getButtonWaitFall(Button sw) {
	if (sw == ButtonLeft) {
		if (btn_states[BTN_LEFT]) {
			while (btn_states[BTN_LEFT]);

			return true;
		}

		return false;
	} else {
		if (btn_states[BTN_RIGHT]) {
			while (btn_states[BTN_RIGHT]);

			return true;
		}

		return false;
	}
}

Joystick getJoystick() {
	if (btn_states[BTN_DPAD_LEFT]) {
		return JoystickLeft;
	}

	if (btn_states[BTN_DPAD_RIGHT]) {
		return JoystickRight;
	}

	if (btn_states[BTN_DPAD_UP]) {
		return JoystickUp;
	}

	if (btn_states[BTN_DPAD_DOWN]) {
		return JoystickDown;
	}

	if (btn_states[BTN_DPAD_CENTER]) {
		return JoystickCenter;
	}

	return JoystickNone;
}

uint16_t getTrimpot(Trimpot adc) {
	ADMUX &= ~7;
	
	if (adc == TrimpotRight) {
		ADMUX |= 1;
	} else if (adc != TrimpotLeft) {
		return 0;
	}

	// Prescale x64
	ADCSRA |= (1 << ADSC);

	while (ADCSRA & (1 << ADSC));

	return ADC;
}

void setLcdBacklight(bool value) {
	if (value) {
		PORTC &= ~(1 << PC7);
	} else {
		PORTC |= 1 << PC7;
	}
}

static inline void restart() {
	cli();

	UDCON = 1;
	USBCON = (1 << FRZCLK);
	UCSR1B = 0;

	_delay_ms(5);

	EIMSK = 0;
	PCICR = 0;
	SPCR = 0;
	ACSR = 0;
	EECR = 0;
	ADCSRA = 0;
	TIMSK0 = 0;
	TIMSK1 = 0;
	TIMSK3 = 0;
	TIMSK4 = 0;
	UCSR1B = 0;
	TWCR = 0;
	DDRB = 0;
	DDRC = 0;
	DDRD = 0;
	DDRE = 0;
	DDRF = 0;
	TWCR = 0;
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;
	PORTE = 0;
	PORTF = 0;

	asm volatile("jmp 0x7e00");
}

#define LCD_EXTENDED(x) { lcd_write(LCD_C, 0x21); (x); lcd_write(LCD_C, 0x20); }

void setLcdContrast(Contrast value) {
	uint8_t contrast = 0;

	switch (value) {
	case ContrastLow:
		contrast = LCD_LOW_CONTRAST;
		break;
	case ContrastDefault:
		contrast = LCD_DEFAULT_CONTRAST;
		break;
	case ContrastHigh:
		contrast = LCD_HIGH_CONTRAST;
		break;
	default:
		return;
	}

	LCD_EXTENDED({
		lcd_write(LCD_C, 0x80 | contrast);
	});
}

void setLcdInversion(bool invert) {
	if (invert) {
		lcd_write(LCD_C, 0x0d);
	} else {
		lcd_write(LCD_C, 0x0c);
	}
}

#pragma endregion

#pragma region Menus

typedef void(*ProgramFn)();

struct tagProgram {
	char* name;
	ProgramFn program;
	bool endOnExit;
};

typedef struct tagProgram Program;

int showMenu(char* title, int programCount, Program programs[]) {
	int offset = 0;
	int selected = 0;
	int keepGoing = 1;

	while (keepGoing) {
		clear_screen();

		scrDrawString((LCD_X - 1 - strlen(title) * 5) / 2, 2, title, false);

		scrDrawRect(0, 11, LCD_X - 1, 46, true, true);

		int stop = min(programCount, offset + 4);

		for (int i = offset, j = 0; i < stop; i++) {
			char buffer[128];

			sprintf(buffer, "%s", programs[i].name);

			int vert_offset = 8 * (j++) + 13;

			scrDrawRect(2, vert_offset, LCD_X - 3, vert_offset + 8, false, selected == i);
			scrDrawString(2, vert_offset, buffer, selected == i);
		}

		int joystick = getJoystick();

		if (joystick == JoystickDown) {
			++selected;
		} else if (joystick == JoystickUp) {
			--selected;
		}

		if (selected < 0) {
			selected = 0;
		} else if (selected >= programCount) {
			selected = programCount - 1;
		}

		if (selected < offset) {
			--offset;
		} else if (selected >= stop) {
			++offset;
		}

		if (getButtonWaitFall(ButtonRight)) {
			if (programs[selected].program != NULL) {
				programs[selected].program();
			}

			setLed(LedLeft, false);
			setLed(LedRight, false);

			if (programs[selected].endOnExit) {
				keepGoing = 0;
			}
		}

		_delay_ms(16);

		//write the string on the lcd
		show_screen();
	}

	return selected;
}

#pragma endregion

#pragma region Programs

void teensy_debug() {
	while (true) {
		clear_screen();

		switch (getJoystick()) {
		case JoystickNone:
			scrDrawString(0, 0, "JOY: None", false);
			break;
		case JoystickUp:
			scrDrawString(0, 0, "JOY: Up", false);
			break;
		case JoystickRight:
			scrDrawString(0, 0, "JOY: Right", false);
			break;
		case JoystickDown:
			scrDrawString(0, 0, "JOY: Down", false);
			break;
		case JoystickLeft:
			scrDrawString(0, 0, "JOY: Left", false);
			break;
		case JoystickCenter:
			scrDrawString(0, 0, "JOY: Center", false);
			break;
		}

		if (getButton(ButtonLeft)) {
			scrDrawString(0, 8, "SW2: On", false);
			setLed(LedLeft, true);
		} else {
			scrDrawString(0, 8, "SW2: Off", false);
			setLed(LedLeft, false);
		}

		if (getButton(ButtonRight)) {
			scrDrawString(0, 16, "SW3: On", false);
			setLed(LedRight, true);
		} else {
			scrDrawString(0, 16, "SW3: Off", false);
			setLed(LedRight, false);
		}

		{
			char buffer[15];

			sprintf(buffer, "OVF: %lu", mstime());

			scrDrawString(0, 24, buffer, false);
		}

		{
			char buffer[17];

			sprintf(buffer, "POT0: %d", getTrimpot(TrimpotLeft));

			scrDrawString(0, 32, buffer, false);
		}

		{
			char buffer[17];

			sprintf(buffer, "POT1: %d", getTrimpot(TrimpotRight));

			scrDrawString(0, 40, buffer, false);
		}

		show_screen();

		_delay_ms(16);
	}
}

static inline void notImplemented() {
	clear_screen();

	scrDrawRect(0, 13, LCD_X - 1, 33, true, true);
	scrDrawString(2, 20, "Not implemented.", false);

	show_screen();

	while (!getButtonWaitFall(ButtonLeft));
}

void snake() {
	notImplemented();
}

#pragma endregion

#pragma region Tutorials

void tutorials() {
	Program programs[] = {
		{ "Exit", NULL, true }
	};

	showMenu("Tutorials", sizeof(programs) / sizeof(Program), programs);
}

#pragma endregion

#pragma region Settings

static inline void backlightOn() {
	setLcdBacklight(true);
}

static inline void backlightOff() {
	setLcdBacklight(false);
}

static inline void contrastLow() {
	setLcdContrast(ContrastLow);
}

static inline void contrastNormal() {
	setLcdContrast(ContrastDefault);
}

static inline void contrastHigh() {
	setLcdContrast(ContrastHigh);
}

static inline void invertOn() {
	setLcdInversion(true);
}

static inline void invertOff() {
	setLcdInversion(false);
}

void settings() {
	Program programs[] = {
		{ "Backlight ON", &backlightOn, false },
		{ "Backlight OFF", &backlightOff, false },
		{ "Contrast LOW", &contrastLow, false },
		{ "Contrast NORMAL", &contrastNormal, false },
		{ "Contrast HIGH", &contrastHigh, false },
		{ "Invert ON", &invertOn, false },
		{ "Invert OFF", &invertOff, false },
		{ "Exit", NULL, true }
	};

	showMenu("Settings", sizeof(programs) / sizeof(Program), programs);
}

#pragma endregion

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

void load() {
	uint32_t time = mstime();
	double amt = 0.0;

	while (amt < 1.0) {
		uint32_t ctime = mstime();
		double delta = (double)ctime - (double)time;

		amt += delta / 500.0;

		time = ctime;

		clear_screen();

		int8_t tlx = (LCD_X - 1 - 64) / 2;
		int8_t tly = (LCD_Y - 1 - 18) / 2;

		scrDrawRect(tlx, tly, tlx + 64, tly + 8, true, true);
		scrDrawRect(tlx + 2, tly + 2, tlx + (62 * cap(amt, 0.0, 1.0)), tly + 7, false, true);

		scrDrawString((LCD_X - 1 - 50) / 2, tly + 10, "Loading...", false);

		show_screen();

		_delay_ms(16);
	}
}

int main(void) {
	setup();
	load();

	Program programs[] = {
		{ "Debug", &teensy_debug, false },
		{ "Snake", &snake, false },
		{ "Tutorials", &tutorials, false },
		{ "Settings", &settings, false },
		{ "Restart", &restart, false }
	};

	showMenu("Programs", sizeof(programs) / sizeof(Program), programs);

	clear_screen();

	scrDrawString(24, 20, "Goodbye!", false);

	show_screen();

	return 0;
}

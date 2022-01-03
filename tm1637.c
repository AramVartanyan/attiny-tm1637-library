/**
 * Copyright (c) 2017-2018, Łukasz Marcin Podkalicki <lpodkalicki@gmail.com>
 * Updated by AramVartanyan 01-2022
 *
 * This is ATtiny13/25/45/85 library for 4-Digit LED Display based on TM1637 chip.
 *
 * Features:
 * - display raw segments
 * - display digits
 * - display colon
 * - display on/off
 * - brightness control
 *
 * References:
 * - library: https://github.com/lpodkalicki/attiny-tm1637-library
 * - documentation: https://github.com/lpodkalicki/attiny-tm1637-library/README.md
 * - TM1637 datasheet: https://github.com/lpodkalicki/attiny-tm1637-library/blob/master/docs/TM1637_V2.4_EN.pdf
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "tm1637.h"
#include "math.h"

#define	TM1637_DIO_HIGH()		(PORTB |= _BV(TM1637_DIO_PIN))
#define	TM1637_DIO_LOW()		(PORTB &= ~_BV(TM1637_DIO_PIN))
#define	TM1637_DIO_OUTPUT()		(DDRB |= _BV(TM1637_DIO_PIN))
#define	TM1637_DIO_INPUT()		(DDRB &= ~_BV(TM1637_DIO_PIN))
#define	TM1637_DIO_READ() 		(((PINB & _BV(TM1637_DIO_PIN)) > 0) ? 1 : 0)
#define	TM1637_CLK_HIGH()		(PORTB |= _BV(TM1637_CLK_PIN))
#define	TM1637_CLK_LOW()		(PORTB &= ~_BV(TM1637_CLK_PIN))

static void TM1637_send_config(const uint8_t enable, const uint8_t brightness);
static void TM1637_start(void);
static void TM1637_stop(void);
static uint8_t TM1637_write_byte(uint8_t value);

static uint8_t _config = TM1637_SET_DISPLAY_ON | TM1637_BRIGHTNESS_MAX;
static uint8_t _segments = 0xff;
PROGMEM const uint8_t _digit2segments[] =
{
	0x3F, // 0
	0x06, // 1
	0x5B, // 2
	0x4F, // 3
	0x66, // 4
	0x6D, // 5
	0x7D, // 6
	0x07, // 7
	0x7F, // 8
	0x6F  // 9
};

void
TM1637_init(const uint8_t enable, const uint8_t brightness)
{

	DDRB |= (_BV(TM1637_DIO_PIN)|_BV(TM1637_CLK_PIN));
	PORTB &= ~(_BV(TM1637_DIO_PIN)|_BV(TM1637_CLK_PIN));
	TM1637_send_config(enable, brightness);
    
    TM1637_DIO_HIGH();
    TM1637_CLK_HIGH();
}

void
TM1637_enable(const uint8_t value)
{

	TM1637_send_config(value, _config & TM1637_BRIGHTNESS_MAX);
}

void
TM1637_set_brightness(const uint8_t value)
{

	TM1637_send_config(_config & TM1637_SET_DISPLAY_ON,
		value & TM1637_BRIGHTNESS_MAX);
}

void
TM1637_display_segments(const uint8_t position, const uint8_t segments)
{
    TM1637_start();
    TM1637_write_byte(TM1637_CMD_FIXED_ADDR);
    TM1637_stop();
    
	TM1637_start();
	TM1637_write_byte(TM1637_CMD_SET_ADDR | (position & (TM1637_POSITION_MAX - 1)));
	TM1637_write_byte(segments);
	TM1637_stop();
}

static void
TM1637_set_segment_raw(const uint8_t position, const uint8_t data) {
    TM1637_start();
    TM1637_write_byte(TM1637_CMD_FIXED_ADDR);
    TM1637_stop();
    
    TM1637_start();
    TM1637_write_byte(position | TM1637_CMD_SET_ADDR);
    TM1637_write_byte(data);
    TM1637_stop();
}

void
TM1637_display_digit(const uint8_t position, const uint8_t digit)
{
	uint8_t segments = (digit < 10 ? pgm_read_byte_near((uint8_t *)&_digit2segments + digit) : 0x00);

	if (position == 0x01) {
		segments = segments | (_segments & 0x80);
		_segments = segments;
	}

	TM1637_display_segments(position, segments);
}

void
TM1637_display_colon(const uint8_t value)
{

	if (value) {
		_segments |= 0x80;
	} else {
		_segments &= ~0x80;
	}
	TM1637_display_segments(0x01, _segments);
}

void
TM1637_display_float(float f_value)
{

    bool msign = false;
    uint8_t seg_data = 0x00;
    uint8_t int_part = 0x00; //digits count before decimal point
    int precision = 0x00; //digits after decimal point
    int i;
    
    if (f_value < 0) {
        f_value = f_value * (-1);
        msign = true;
    }
    
    if (f_value < 10) {
        int_part = 1;
    } else if (f_value < 100) {
        int_part = 2;
    } else if (f_value < 1000) {
        int_part = 3;
    } else {
        int_part = 4;
    }
    
    precision = 4 - int_part;
    
    if (msign) {
        precision = precision - 1;
    }
    
    f_value = f_value * pow(10,precision);
    
    uint8_t dp = 3 - precision;
    uint16_t num = f_value;
    uint16_t sum = 0;
    uint8_t f_num[3];
    
    for (i = 0; i < 3; ++i) {
        if (i == 0 && msign == true) {
            seg_data = 0x40; //minus sign
            TM1637_set_segment_raw(i, seg_data);
        } else {
            f_num[i] = (uint8_t)((num - sum) / pow(10,(3 - i)));
            sum = sum + f_num[i] * pow(10,(3 - i));
            //seg_data = tm_num[f_num[i]];
            seg_data = pgm_read_byte_near((uint8_t *)&_digit2segments + f_num[i]);
            if (i == dp && precision > 0) {
                seg_data |= 0x80; //decimal point
            }
            TM1637_set_segment_raw(i, seg_data);
        }
    }
}

void
TM1637_clear(void)
{
	uint8_t i;

	for (i = 0; i < TM1637_POSITION_MAX; ++i) {
		TM1637_display_segments(i, 0x00);
	}
}

void
TM1637_send_config(const uint8_t enable, const uint8_t brightness)
{

	_config = (enable ? TM1637_SET_DISPLAY_ON : TM1637_SET_DISPLAY_OFF) |
		(brightness > TM1637_BRIGHTNESS_MAX ? TM1637_BRIGHTNESS_MAX : brightness);
    
    TM1637_start();
    TM1637_write_byte(TM1637_CMD_SET_DSIPLAY | _config);
    TM1637_stop();
}

void
TM1637_start(void)
{
    // Send start signal
    // Both outputs are expected to be HIGH beforehand
    TM1637_DIO_LOW();
	_delay_us(TM1637_DELAY_US);
}

void
TM1637_stop(void)
{
    // Send stop signal
    // CLK is expected to be LOW beforehand
    
    TM1637_DIO_LOW();
    _delay_us(TM1637_DELAY_US);
    
    TM1637_CLK_HIGH();
    _delay_us(TM1637_DELAY_US);
    
    TM1637_DIO_HIGH();
    _delay_us(TM1637_DELAY_US);
}

uint8_t
TM1637_write_byte(uint8_t byte)
{
	for (uint8_t i = 0; i < 8; ++i, byte >>= 1) {
		TM1637_CLK_LOW();
		_delay_us(TM1637_DELAY_US);

		if (byte & 0x01) {
			TM1637_DIO_HIGH();
		} else {
			TM1637_DIO_LOW();
		}
        _delay_us(TM1637_DELAY_US); // Check if could work without it? ...at 3us

		TM1637_CLK_HIGH();
		_delay_us(TM1637_DELAY_US);
	}

    /*
     The TM1637 signals an ACK by pulling DIO low from the falling edge of CLK
     after sending the 8th bit, to the next falling edge of CLK.
     DIO needs to be set as input during this time to avoid
     having both chips trying to drive DIO at the same time.
     */
    
    TM1637_CLK_LOW(); // TM1637 starts ACK (pulls DIO low)
    _delay_us(TM1637_DELAY_US);
    
    TM1637_DIO_INPUT();

    uint8_t ack = TM1637_DIO_READ();
    
    //while (ack) { ack = TM1637_DIO_READ(); }
    
    for (i = 0; i < 3; ++i) {
        
        if (ack) {
            _delay_us(TM1637_DELAY_US);
            ack = TM1637_DIO_READ();
        } else {
            i = 3;
        }
    }
    
    TM1637_CLK_HIGH();
    _delay_us(TM1637_DELAY_US);
    
    TM1637_CLK_LOW(); // TM1637 ends ACK (releasing DIO)
    _delay_us(TM1637_DELAY_US);
    
    TM1637_DIO_OUTPUT();

	return ack;
}

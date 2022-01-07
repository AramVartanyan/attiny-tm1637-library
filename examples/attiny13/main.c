/**
 * Copyright (c) 2017-2018, Łukasz Marcin Podkalicki <lpodkalicki@gmail.com>
 *
 * This is ATtiny13 "Running Digits" example using attiny-tm1637-library,
 * https://github.com/lpodkalicki/attiny-tm1637-library .
 *
 */

#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include "tm1637.h"

int
main(void)
{
	uint8_t n, k = 0;

	/* setup */
	TM1637_init(1/*enable*/, 5/*brightness*/);

	/* loop */
	while (1) {
        
        // Test print float
        float test_number = 99.99;
        while (test_number > - 99.99) {
            if (test_number > 5) {
                test_number = test_number - 1.234;
            } else if (test_number < 5 && test_number > - 5) {
                test_number = test_number - 0.123;
            } else {
                test_number = test_number - 5.678;
            }
            TM1637_display_float(test_number);
            _delay_ms(200);
        }
        
		for (n = 0; n < TM1637_POSITION_MAX; ++n) {
			TM1637_display_digit(n, (k + n) % 0x10);
		}
		TM1637_display_colon(1);
		_delay_ms(200);
		TM1637_display_colon(0);
		_delay_ms(200);
		k++;
	}
}

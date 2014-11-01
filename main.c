/*
 * Copyright Brian Starkey 2014 <stark3y@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#define F_CPU 8000000

#define DEBUG

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "i2c/i2c_machine.h"
#include "i2c/i2c_slave_defs.h"

#define LED_ON() PORTB |= 0x2
#define LED_OFF() PORTB &= ~0x2
#define LED_FLICKER() LED_OFF(); LED_ON()

#define ADCMUX_MASK 0x1F
#define N_CHANNELS  10

volatile uint8_t i2c_reg[I2C_N_REG] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
const uint8_t i2c_w_mask[I2C_N_REG] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

void adc_init(void)
{
	/* Left adjust result */
	ADMUX = (1 << ADLAR);
	/* CK / 64 = 125 kHz
	 * @13 cycles per sample, about 10 kHz sample rate. 100 us/sample  */
	ADCSR = (1 << ADEN) | (0 << ADIE) | (6 << ADPS0);

}

int main(void)
{
	uint8_t channel = 0;
	uint8_t skip = 1;
	DDRB = 0x02;
	PORTB = 0x00;
	DDRA = 0;
	PORTA = 0;

#ifdef DEBUG
	LED_ON();
	_delay_ms(30);
	LED_OFF();
	_delay_ms(30);
	LED_ON();
	_delay_ms(30);
	LED_OFF();
#endif

	i2c_init();
	adc_init();
	sei();

	ADCSR &= ~ADCMUX_MASK;
	ADMUX |= channel << MUX0;
	ADCSR |= (1 << ADIF) | (1 << ADSC);
	while (1) {
		i2c_check_stop();

		if (ADCSR & (1 << ADIF)) {
			if (!skip) {
				i2c_reg[channel] = ADCH;
				channel++;
				if (channel == N_CHANNELS) {
					channel = 0;
				}
				ADMUX &= ~ADCMUX_MASK;
				ADMUX |= (channel << MUX0);
			}
			skip ^= 1;
			ADCSR |= (1 << ADIF) | (1 << ADSC);
		}
	}

	return 0;
}

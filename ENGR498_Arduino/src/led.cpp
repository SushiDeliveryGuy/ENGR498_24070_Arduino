#include <avr/io.h>
#include <util/delay.h>
#include "led.h"

// Initialize 3 Port outputs for LEDS (green, yellow, red)
void initLED() {
    DDRB |= (1 << PORTB0) | (1 << PORTB1) | (1 << PORTB2);
}

void turnOnLEDWithChar(unsigned char pattern) {
    // Pattern is 3 bits representing the LEDs ("100", "010", "001")
    if (pattern == 0b100) { // GREEN
        PORTB = (PORTB & 0xF8) | 0x01;
    } else if (pattern == 0b010) { // YELLOW
        PORTB = (PORTB & 0xF8) | 0x02;
    } else if (pattern == 0b001) { // RED
        PORTB = (PORTB & 0xF8) | 0x04;
    } else { // Invalid pattern, turn off all LEDs
        PORTB &= 0xF8;
    }
}
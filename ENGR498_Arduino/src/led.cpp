#include <avr/io.h>
#include <util/delay.h>
#include "led.h"


// Initialize 3 Port outputs for LEDS (green, yellow, red)
void initLED() {
    DDRD |= (1 << PORTD0) | (1 << PORTD1) | (1 << PORTD2);
}

void turnOnLEDWithChar(unsigned char pattern) {
    // Pattern is 3 bits representing the LEDs ("100", "010", "001")
    if (pattern == 0b100) { // GREEN
        PORTD = (PORTD & 0xF8) | 0x01;
    } else if (pattern == 0b010) { // YELLOW
        PORTD = (PORTD & 0xF8) | 0x02;
    } else if (pattern == 0b001) { // RED
        PORTD = (PORTD & 0xF8) | 0x04;
    } else { // Invalid pattern, turn off all LEDs
        PORTD &= 0xF8;
    }
}
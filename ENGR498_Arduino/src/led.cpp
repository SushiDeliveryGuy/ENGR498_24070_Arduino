#include <avr/io.h>
#include <util/delay.h>
#include "led.h"


// Initialize 3 Port outputs for LEDS (green, yellow, red)
void initLED() {
    DDRD |= (1 << PORTD0) | (1 << PORTD1) | (1 << PORTD2);
}


// TODO: CHANGE "turnOnLEDWithChar" (ONLY ONE LIGHT NEEDED AT A TIME)

// given binary representation "num" the lowest four bits of num will be assigned to the appropriate bits of PORTD (LED ports)
void turnOnLEDWithChar(unsigned char num) {
    PORTD = (PORTD & 0xF0) | (num & 0x0F);
}
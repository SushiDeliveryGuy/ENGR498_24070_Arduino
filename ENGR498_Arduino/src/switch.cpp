#include "switch.h"
#include <avr/io.h>

void initSwitchPD0(){
    //data direction
    DDRD &= ~(1 << PD0);
    //port
    PORTD |= (1 << PD0);
    //Set the external interrupt control register A to 11 for INT0
    EICRA |= (1<<ISC00) | (1<<ISC01);
    //turn on INT0 in the external interrupt mask register to enable it
    EIMSK |= (1<<INT0);
}

void initSwitchPD1() {
    DDRD &= ~(1 << PD1);
    PORTD |= (1 << PD1);
}
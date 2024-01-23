#include "switch.h"
#include <avr/io.h>

/*
 * Initializes pull-up resistor on PB3 and sets it into input mode
 */
void initSwitchPD0(){
  //data direction
DDRB &= ~(1 << PD0);
//port
PORTB |= (1 << PD0);

//Set the external interrupt control register A to 11 for INT0
EICRA |= (1<<ISC00) | (1<<ISC01);
//turn on INT0 in the external interrupt mask register to enable it
EIMSK |= (1<<INT0);




}

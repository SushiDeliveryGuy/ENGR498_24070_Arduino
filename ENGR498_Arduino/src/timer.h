#ifndef TIMER_H
#define TIMER_H

#include <avr/io.h>

void initTimer1();
void delayMs1(unsigned int delay);
void initTimer0();
void delayMs0(unsigned int delay); 

#endif

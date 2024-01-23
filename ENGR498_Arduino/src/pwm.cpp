#include <avr/io.h>
#include <util/delay.h>
#include "pwm.h"

void initPWMTimer3(){

//initialize pin
    DDRB |= (1 << DDB5);
//Set timer counter control register to fast PWM 10-bit mode
    TCCR3A |= (1 << WGM30);
    TCCR3A |= (1 << WGM31);
    TCCR3B |= (1 << WGM32);
    TCCR3B &= ~(1 << WGM33);
// set timer counter control register to prescaler of 8
    TCCR3B |= (1 << CS30); 
    TCCR3B &= ~(1 << CS31);
    TCCR3B &= ~(1 << CS32);
// set compare output mode 
    TCCR3A &= ~(1 << COM3A0);
    TCCR3A |= (1 << COM3A1);

    OCR3A = 255;



    
}

void changeDutyCycle(unsigned int combo){

OCR3A = combo;

}
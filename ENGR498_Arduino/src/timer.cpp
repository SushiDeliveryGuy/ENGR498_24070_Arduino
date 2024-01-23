#include "timer.h"
//You many use any timer you wish for the microsecond delay and the millisecond delay


/* Initialize timer 1, you should not turn the timer on here. Use CTC mode  .*/


void initTimer1(){

//Set timer counter control register to CTC mode
    TCCR1A &= ~(1 << WGM10);
    TCCR1A &= ~(1 << WGM11);
    TCCR1B |= (1 << WGM12);
    TCCR1B &= ~(1 << WGM13);
// set timer counter control register to prescaler of 64
    TCCR1B |= (1 << CS10); 
    TCCR1B |= (1 << CS11);
    TCCR1B &= ~(1 << CS12);

//Set output compare register to 250
    OCR1AH = 0;
    OCR1AL = 250;



    
}

/* This delays the program an amount of miliseconds specified by unsigned int delay.
*/
void delayMs1(unsigned int delay){
    //initialize delay count
    unsigned int delayCnt = 0;

//initialize timer at 0
TCNT1 = 0;

//set compare flag  to start timer (flag down is logic 1)
TIFR1 |= (1 << OCF1A);
//while loop for creating delay
while (delayCnt < delay){
//check if OCF0A is 0 (flag is set)
if (TIFR1 & (1 << OCF1A)){
delayCnt++;
TIFR1 |= (1 << OCF1A);
}
}
}

/* Initialize timer 0, you should not turn the timer on here.
* You will need to use CTC mode */
void initTimer0(){

// set CTC mode prescaler = 64
TCCR0A &= ~(1 << WGM00);
TCCR0A |= (1 << WGM01);
TCCR0B &= ~(1 << WGM02);



//011 from prescaler 64
TCCR0B |= (1 << CS01) | (1 << CS00);
TCCR0B &= ~(1 << CS02);
//OCR0A for prescaler 64
OCR0A = 250;

}

/* This delays the program an amount specified by unsigned int delay.
* Use timer 0. Keep in mind that you need to choose your prescalar wisely
* such that your timer is precise to 1 millisecond and can be used for
* 100-200 milliseconds
*/
void delayMs0(unsigned int delay){
    //initialize delay count
    unsigned int delayCnt = 0;

//initialize timer at 0
TCNT0 = 0;
//set compare flag  to start timer (flag down is logic 1)
TIFR0 |= (1 << OCF0A);
//while loop for creating delay
while (delayCnt < delay){
//check if OCF0A is 0 (flag is set)
if (TIFR0 & (1 << OCF0A)){
delayCnt++;
TIFR0 |= (1 << OCF0A);

}
}
}
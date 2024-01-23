#include <avr/io.h>
#include <util/delay.h>
#include "adc.h"

void initADC(){
    //data direction for port A0
    DDRA &= ~(1 << PA0);

    //set input channel ADC0, sets mode to differential and gain to 10x
    ADMUX |= (1 << MUX3);

    //sets vref to internal 2.56 reference voltage
    ADMUX |= (1 << REFS1) | (1<< REFS0);


    //sets ADCH and ADCL to left adjusted
    ADMUX |= (1 << ADLAR);

    //set auto trigger to disable
    ADMUX &= ~(1 << ADATE);

    //set interrupt to disable
    ADMUX &= ~(1 << ADIE);

    //enables the ADC
    ADCSRA |= (1 << ADEN);

    //start a conversion
    ADCSRA |= (1<<6);
}
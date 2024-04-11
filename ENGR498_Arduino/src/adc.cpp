#include <avr/io.h>
#include <util/delay.h>
#include "adc.h"

void initADC_A0(){
    //set input channel ADC0, sets mode to single ended input with MUX[5:0] = 0b000000
    ADMUX &= ~(1 << MUX2) & ~(1 << MUX1) & ~(1 << MUX0);
    ADMUX &= ~((1 << MUX4) | (1 << MUX3));
    ADCSRB &= ~(1 << MUX5);
    //sets vref to AVCC with external capaciter at AREF pin
    ADMUX &= ~(1 << REFS1);
    ADMUX |= (1<< REFS0);
    //sets ADCH and ADCL to right adjusted
    ADMUX &= ~(1 << ADLAR);
    //set Auto Trigger Source Selection. Use free-running mode ADTS[2:0] = 0b000
    ADCSRB &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));
    //set interrupt to disable
    ADMUX &= ~(1 << ADIE);
    // set the ADC clock frequency.  Use a pre-scaler of 128  
    //(ADC clock frequency is 16 Mhz divided by pre-scaler = 125KHz,Sampling rate is 1/ ((1/125K Hz )*(13 clock cycles)) = 9615 KHz)
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    //enables the ADC and auto-triggering
    ADCSRA |= (1 << ADEN) | (1 << ADATE);
    //disable ADC0 pin digital input - pin A0 on board
    DIDR0 |= (1 << ADC0D);
    //start a conversion
    ADCSRA |= (1<<ADSC);
}

void initADC_A1(){
    //set input channel ADC1, sets mode to single ended input with MUX[5:0] = 0b000000
    // ADMUX &= ~(1 << MUX2) & ~(1 << MUX1) & ~(1 << MUX0);
    // ADMUX &= ~((1 << MUX4) | (1 << MUX3));
    // ADCSRB &= ~(1 << MUX5);
    ADMUX |= (1 << MUX0);
    ADMUX &= ~((1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1));
    ADCSRB &= ~(1 << MUX5);
    //sets vref to AVCC with external capaciter at AREF pin
    ADMUX &= ~(1 << REFS1);
    ADMUX |= (1<< REFS0);
    //sets ADCH and ADCL to right adjusted
    ADMUX &= ~(1 << ADLAR);
    //set Auto Trigger Source Selection. Use free-running mode ADTS[2:0] = 0b000
    ADCSRB &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));
    //set interrupt to disable
    ADMUX &= ~(1 << ADIE);
    // set the ADC clock frequency.  Use a pre-scaler of 128  
    //(ADC clock frequency is 16 Mhz divided by pre-scaler = 125KHz,Sampling rate is 1/ ((1/125K Hz )*(13 clock cycles)) = 9615 KHz)
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    //enables the ADC and auto-triggering
    ADCSRA |= (1 << ADEN) | (1 << ADATE);
    //disable ADC0 pin digital input - pin A1 on board
    DIDR0 |= (1 << ADC1D);
    //start a conversion
    ADCSRA |= (1<<ADSC);
}


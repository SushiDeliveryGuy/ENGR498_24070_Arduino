#include <Arduino.h>
#include <avr/io.h>
#include "switch.h"
#include "timer.h"
#include "pwm.h"
#include "led.h"


/*
  Declares states from state machine:
    idle - waiting at base
    sample - moving/taking sample
    e_stop - return to base and stay until wait state left
    wait - post e_stop idle state
*/
typedef enum {idle, sample, e_stop, wait} states;
states arm = idle;

int main() {

  initLED();
  initTimer0();
  initTimer1();
  initPWMTimer3();
  initSwitchPD0();

  sei(); // enables global interrupts

  while(1) {

    switch(arm) {
        case idle: // handler for idle state
            // Wait for sample switch button
            // PIN_BUTTON --> Sample button
            // DIAL_SPEED --> Potentiometer value --> duty cycle based on voltage value
            /*
            grn = "100"; 
            turnOnLEDWithChar(grn); // turn on green LED
            changeDutyCycle(DIAL_SPEED);
            if (PIN_BUTTON pressed) {
              
              arm = sample;
            }
            */
          break;
        case sample: // handler for sample state
            // Motor movement with timing(position sensor data)
            // Speed based on dial --> Duty Cycle based on input (Should be set in idle case handler)
            /*

            yel = "010";
            turnOnLEDWithChar(yel); // turn on yellow LED (switch off other LEDs)
            while(SENSOR_POSITION <= MAX_DISTANCE) {
              MOTOR FORWARD CODE
            }
            while (SENSOR_POSITION >= 0) {
              MOTOR BACKWARDS CODE
            }
            */
          break;
        case wait: // handler for wait state
            while (1) {
              
            }
          break;
        default:
          break;
    }
  }
  return 0;
}

// Interrupt Handler (E-Stop Button)
ISR(INT0_vect) {
  if (!(PIND & (1 << PIND0))) {
    /* Logic for blinking red LED during E-Stop procedures
    char red = '001';
    turnOnLEDWithChar(red);
    */
    if (arm == sample) {
        /* E-Stop Procedures 
        while (SENSOR_LOCATION >= 0 (WITHIN A MARGIN?) ) {
          MOTOR BACKWARDS CODE
        }
        while (1) { // SHOULD NOT ALLOW FOR OPERATIONS UNTIL CLEARED.

        }
        */
        arm = wait;
        // turnOnLEDWithChar(red); // Solid red light after E-procedures are complete
    }
    else if (arm == wait) { // pressing the e-stop button again should exit the "wait to clear" mode and return to idle
      arm = idle;
    }
  }
}
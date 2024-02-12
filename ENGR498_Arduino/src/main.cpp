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

const int speed_pot_PIN = A0;     // SPEED DIAL WIRED TO PINA0
const int distance_pot_PIN = A1;  // DISTANCE DIAL WIRED TO PINA1

const int direction_forward_PIN = A2; // DIGITAL OUTPUT FOR H-BRIDGE TO CONTROL MOTOR DIRECTION
const int direction_reverse_PIN = A3; // DIGITAL OUTPUT FOR H-BRIDGE TO CONTROL MOTOR DIRECTION

const int position_sensor_PIN = A4; // ANALOG INPUT OF POSITION SENSOR DATA

const int PIN_BUTTON = PIND1;     // TAKE SAMPLE BUTTON PIND1

// TODO: TESTING TO SET SAFETY MARGIN ON DISTANCE
const int MIN_DISTANCE = 1;       // MINIMUM ARM DISTANCE
const int MAX_DISTANCE = 10;      // MAXIMUM ARM DISTANCE
const int DISTANCE_MARGIN = MAX_DISTANCE * 0.1; // POSITION MARGIN IS 10% OF MAX DISTANCE (TBD)
//TODO: TESTING TO SET DEFUALT RETURN MOTOR SPEED
const int RETURN_SPEED = 125;     // "REASONABLY SAFE" SPEED FOR E-STOP PROCEDURES

int main() {

  initLED();
  initTimer0();
  initTimer1();
  initPWMTimer3();
  initPWMTimerSpeed();
  initSwitchPD0();
  initSwitchPD1();

  sei(); // enables global interrupts

  pinMode(position_sensor_PIN, INPUT);

  while(1) {

    switch(arm) {
        case idle: // handler for idle state
            turnOnLEDWithChar(0b100); // SOLID GREEN LED STATUS
            unsigned int DIAL_SPD_MEASURE = analogRead(speed_pot_PIN);  // Reads potentiometer value (0-1023)
            unsigned int DIAL_SPEED = map(DIAL_SPD_MEASURE, 0, 1023, 0, 255); // translates the potentiometer value to PWM range
// TODO: TESTING TO SET SAFETY MARGIN ON MOTOR SPEED
           

            unsigned int DIAL_DIST_MEASURE = analogRead(distance_pot_PIN); // see DIAL_SPD_MEASURE initialization
            unsigned int DIAL_DISTANCE = map(DIAL_DIST_MEASURE, 0, 1023, 0, 255);
            unsigned int SET_DISTANCE = DIAL_DISTANCE;
            if (SET_DISTANCE >= MAX_DISTANCE) { // Depending on the range of the dial, the distance will still not exceed the max distance
              SET_DISTANCE = MAX_DISTANCE;
            }
            if (digitalRead(PIN_BUTTON) == LOW) {  // Changes states to "sample" when the button is pressed
              arm = sample;
            }
            break;
        case sample: // handler for sample state
            digitalWrite(direction_forward_PIN, HIGH); // switches direction of motor to forward
            digitalWrite(direction_reverse_PIN, LOW);

            turnOnLEDWithChar(0b010); // SOLID YELLOW LED STATUS
            while(1) { 
              changeDutyCycleSpeed(DIAL_SPEED);    // MOTOR STARTS MOVING (WITH SET SPEED)
              unsigned int SENSOR_DATA = analogRead(position_sensor_PIN);
              unsigned int SENSOR_LOC = map(SENSOR_DATA, 0, 1023, 0, SET_DISTANCE);
              if (SENSOR_LOC >= SET_DISTANCE - DISTANCE_MARGIN) { // when the arm reaches set max distance, switch motor direction.
                changeDutyCycleSpeed(0); // Stops motor
                digitalWrite(direction_forward_PIN, LOW); // switches motor direction to reverse
                digitalWrite(direction_reverse_PIN, HIGH);
                changeDutyCycleSpeed(DIAL_SPEED); // MOTOR STARTS MOVING IN REVERSE DIRECTION
                break;
              }
            }
            while(1) {
              unsigned int SENSOR_DATA = analogRead(position_sensor_PIN);
              unsigned int SENSOR_LOC = map(SENSOR_DATA, 0, 1023, 0, SET_DISTANCE); 
              if (SENSOR_LOC <= MIN_DISTANCE + DISTANCE_MARGIN) {
                changeDutyCycleSpeed(0); // stops motor with dutyCycle = 0
                break;
              }
            }
            arm = idle; // sets the state back to idle
            break;
        case wait: // handler for wait state
            turnOnLEDWithChar(0b001);
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
  if (!(PIND & (1 << PIND0))) { // Interrupt listener pin receives 'HIGH' 
    if (arm == sample) {
      changeDutyCycleSpeed(RETURN_SPEED); // RETURN_SPEED should be a "safe" speed to return the arm back to base
      while(1) {
        turnOnLEDWithChar(0b001); //blinking red LED (1 second cycle)
        delayMs1(500);
        turnOnLEDWithChar(0b000);
        delayMs1(500);
        unsigned int SENSOR_DATA = analogRead(position_sensor_PIN);
        unsigned int SENSOR_LOC = map(SENSOR_DATA, 0, 1023, 0, MAX_DISTANCE);
        if (SENSOR_LOC == MIN_DISTANCE) {
          changeDutyCycleSpeed(0); // stops motor
          break;
        }
      }
      arm = wait;
    }
    else if (arm == wait) { // pressing the e-stop button again should exit the "wait to clear" mode and return to idle
      arm = idle;
    }
  }
}
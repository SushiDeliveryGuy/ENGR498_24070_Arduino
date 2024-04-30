#include <Arduino.h>
#include <avr/io.h>
#include "timer.h"
#include "led.h"

/*
  Declares states from state machine:
    idle - waiting at base
    sample - moving/taking sample
    // e_stop - return to base and stay until wait state left
    wait - post e_stop idle state
*/
typedef enum {idle, sample, e_stop, wait} states;
states arm = idle;

const int R_PWM = 10; // ARDUINO PWM OUTPUT PIN 9; CONNECTED TO RPWM
const int L_PWM = 9;  // ARDUINO PWM OUTPUT PIN 10 ; CONNECTED TO LPWM

const int ROT_A = 18; // Arduino Pin 18
const int ROT_B = 19; // Arduino Pin 19

// variable to hold rotary encoder "position"
volatile int ROT_VAL = 0;


const int E_BUTTON = PD0;       // EMERGENCY STOP BUTTON
const int PIN_BUTTON = PD1;     // TAKE SAMPLE BUTTON PIND1

const int MIN_DISTANCE = 10;       // MINIMUM ROTARY ENCODER VALUE
const int MAX_DISTANCE = 950;      // MAXIMUM ROTARY ENCODER VALUE

const int RETURN_SPEED = 70;     // "REASONABLY SAFE" SPEED FOR E-STOP PROCEDURES

// sampling process flags (forwards and backwards)
bool sample_done;
bool return_trip;

// emergency stop flag for interrupt handling
bool flag;

unsigned int DIAL_SPD_MEASURE;
unsigned int DIAL_SPD;
unsigned int DIAL_DIST_MEASURE;
unsigned int DIAL_DIST;

unsigned int SET_DISTANCE;

unsigned int counter = 0;

unsigned int temp_s = 0;
unsigned int temp_d = 0;


void setup() {

initLED();
  initTimer0();
  initTimer1();
  
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(E_BUTTON, INPUT_PULLUP);

  // configure interrupt pins INT0 and INT1 as falling edge triggers
  EICRA |= (1 << ISC01);
  EICRA |= (1 << ISC11);

  // enable external interrupts
  EIMSK |= (1 << INT1) | (1 << INT0);
  Serial.begin(9600);

  // sets motor control pins to output
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  // sets rotary encoder pins as pull-up inputs
  pinMode(ROT_A, INPUT_PULLUP);
  pinMode(ROT_B, INPUT_PULLUP);
  // Set interrupt trigger for rising edge
  EICRA |= (1 << ISC31) | (1 << ISC30); // Rising edge trigger for INT3
  // Enable INT3 in the external interrupt mask register
  EIMSK |= (1 << INT3);
  
  sei(); // enables global interrupts

  // Emergency State Boolean check initialization
  flag = false;
  
}

void loop() {
    if (!flag) {
    switch(arm) {
        case idle: // handler for idle state
            turnOnLEDWithChar(0b001); // SOLID GREEN LED STATUS
            DIAL_SPD = analogRead(A5);
            DIAL_SPD = map(DIAL_SPD, 0, 1023, 0, 200); // translates the potentiometer value to PWM range

            DIAL_DIST = analogRead(A1);
            DIAL_DIST = map(DIAL_DIST, 0, 1023, 0 ,950);

            // Serial Communication to observe Speed measurement
            if (DIAL_SPD != temp_s) {
              Serial.print("SPEED: ");
              Serial.println(DIAL_SPD);
              temp_s = DIAL_SPD;
            }
            // Serial Communication to observe Distance measurement
            if (DIAL_DIST != temp_d) {
                Serial.print("DIST: ");
                Serial.println(DIAL_DIST);
                temp_d = DIAL_DIST;
            }
            break;

        case sample: // handler for sample state
            Serial.println("SAMPLE TIME");
            sample_done = false;
            turnOnLEDWithChar(0b010); // SOLID YELLOW LED STATUS
            // Loop to send sample bucket out to set distance
            while(!sample_done) {
              analogWrite(L_PWM, 0);
              analogWrite(R_PWM, DIAL_SPD);
              Serial.print("ROT: ");
              Serial.println(ROT_VAL);
              // when the arm reaches set max distance, switch motor direction.
              if (ROT_VAL > DIAL_DIST) { 
                // Full stop
                analogWrite(R_PWM, 0);
                analogWrite(L_PWM, 0);
                sample_done = true; // BUCKET FULLY EXTENDED
                return_trip = false; // BUCKET HAS NOT RETURNED
                // Emergency Stop Check
                if (flag) break;
              }
              // Emergency Stop Check
              if (flag) break;
            }
            // Emergency Stop Check
            if (flag) break;

            // Loop to return sample bucket to initial position
            while(!return_trip) {
              analogWrite(R_PWM, 0);
              analogWrite(L_PWM, DIAL_SPD);
              // when sample is within a distance margin, begin slowing down.
              Serial.print("ROT: ");
              Serial.println(ROT_VAL);
              if (ROT_VAL < MIN_DISTANCE) {
                // stop the motor if the sample has returned fully
                analogWrite(L_PWM, 0);
                analogWrite(R_PWM, 0);
                return_trip = true;
                if (flag) break;
              }              
            }
            // Emergency Stop Check
            if (flag) break;
            // sets the state back to idle
            if (arm != wait) arm = idle; 
            break;

        case wait: // handler for wait state
            turnOnLEDWithChar(0b100);
            break;

        case e_stop:
            flag = true;
            break;

        default:
            arm = idle;
            break;
    }
  // Emergency Stop Procedure (For Mid Sample)
  } else {
    // If sample is moving forwards, stop movement, and return to starting position
    analogWrite(L_PWM, 0);
    analogWrite(R_PWM, 0);
    delayMs1(500);
    analogWrite(L_PWM, RETURN_SPEED);
// Update LED's to inform user of current state
    while(ROT_VAL > 5) {
      turnOnLEDWithChar(0b000); //blinking red LED (1 second cycle)
      delayMs1(500);
      turnOnLEDWithChar(0b001);
      delayMs1(500);
    }
    arm = wait;
    flag = false;
  }
}

// Interrupt Handler (E-Stop Button)
ISR(INT0_vect) {
  sample_done = true;
  return_trip = false;
  if (arm == wait) {
    arm = idle;
    flag = false;
  } else if (arm == idle) {
    arm = wait;
    flag = false;
  } else {
    flag = true;
    arm = wait;
  }
}

// Interrupt Handler (Sample Button)
ISR(INT1_vect) {
    if (arm == idle) {
        arm = sample;
    } else {

    }
}

// initalize interrupt handlers for rotary encoder
ISR(INT3_vect) {
  if (digitalRead(ROT_B) == LOW) {
    ROT_VAL--;
  } else {
    ROT_VAL++;
  }
}

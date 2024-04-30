/*
WHEN TESTING "moduleTest.cpp" DO THE FOLLOWING:
  1. Make sure only rotary encoder and motor/motor power components are connected
  2. All other mains() or setup()/loop() pairs are disabled
  3. "Kill-switch" is readily available during tests
*/
#include <Arduino.h>
#include <avr/io.h>
#include "timer.h"
#include "led.h"

//--------- TEST 1: Single Dial-------------//

// // void setup() {
// //     Serial.begin(9600);
// // }
// // void loop() {
// //     int value = analogRead(A2);
// //     Serial.println(value);
// // }

//--------- TEST 2: Single Button-------------//

// // const int BUTTON_PIN = PD1;

// // void setup() {
// //   Serial.begin(9600);
// //   pinMode(BUTTON_PIN, INPUT_PULLUP);
// // }

// // void loop() {
// //   // Read the state of the button
// //   int buttonState = digitalRead(BUTTON_PIN);

// //   // Check if the button is pressed (active LOW)
// //   if (buttonState == LOW) {
// //     Serial.println("Button pressed");
// //     // Wait for the button to be released before printing again
// //     while (digitalRead(BUTTON_PIN) == LOW) {
// //       // Do nothing, just wait for the button to be released
// //     }
// //     Serial.println("Button released");
// //   }
// // }

//--------- TEST 3: Rotary Encoder Value Counting and Motor Switch Directions-------------//

const int RPWM_Output = 10; // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
const int LPWM_Output = 9; // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)

const int ROT_A = 18; // Arduino Pin 18
const int ROT_B = 19; // Arduino Pin 19

volatile int temp = 1;
volatile int ROT_VAL = 0;

// initalizes interrupt handlers for rotary encoder
void ai3() {
  if (digitalRead(ROT_B) == LOW) {
    ROT_VAL--;
  } else {
    ROT_VAL++;
  }
}

void setup() {
    pinMode(RPWM_Output, OUTPUT);
    pinMode(LPWM_Output, OUTPUT);

    // sets rotary encoder pins as pull-up inputs
    pinMode(ROT_A, INPUT_PULLUP);
    pinMode(ROT_B, INPUT_PULLUP);

    // Set interrupt trigger for rising edge
    EICRA |= (1 << ISC31) | (1 << ISC30); // Rising edge trigger for INT3
    // Enable INT3 in the external interrupt mask register
    EIMSK |= (1 << INT3);

    sei(); // enables global interrupts
    attachInterrupt(digitalPinToInterrupt(ROT_A), ai3, RISING);

    Serial.begin(9600);
}

void loop() {
    Serial.println("POWER");
    // forward rotation
    if (ROT_VAL < 5) { // THRESHOLD 
        // if (LPWM_Output == 100) {
        //   for (int i = 100; i > 10; i = i - 10) {
        //     analogWrite(LPWM_Output, i);
        //     delayMs0(100);
        //   }
        // }
        analogWrite(LPWM_Output, 0);
        analogWrite(RPWM_Output, 70);
    }
    // reverse rotation
    if (ROT_VAL > 400) { // THRESHOLD 50
        // if (RPWM_Output == 100) {
        //   for (int i = 100; i > 10; i = i - 10) {
        //     analogWrite(RPWM_Output, i);
        //     delayMs0(100);
        //   }
        // }
        analogWrite(LPWM_Output, 70);
        analogWrite(RPWM_Output, 0);
    }
}

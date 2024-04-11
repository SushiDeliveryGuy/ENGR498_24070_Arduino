#include <Arduino.h>
#include <avr/io.h>
#include "switch.h"
#include "timer.h"
#include "pwm.h"
#include "led.h"
#include "adc.h"

/*
  Declares states from state machine:
    idle - waiting at base
    sample - moving/taking sample
    e_stop - return to base and stay until wait state left
    wait - post e_stop idle state
*/
typedef enum {idle, sample, e_stop, wait} states;
states arm = idle;

const int R_PWM = 10; // ARDUINO PWM OUTPUT PIN 10; CONNECTED TO RPWM
const int L_PWM = 9;  // ARDUINO PWM OUTPUT PIN 9 ; CONNECTED TO LPWM

const int ROT_A = 18; // Arduino Pin 18
const int ROT_B = 19; // Arduino Pin 19

// variable to hold rotary encoder "position"
volatile int ROT_VAL;

// initalizes interrupt handlers for rotary encoder
void ai3() {
  if (digitalRead(ROT_B) == LOW) {
    ROT_VAL--;
  } else {
    ROT_VAL++;
  }
}


const int PIN_BUTTON = PD1;     // TAKE SAMPLE BUTTON PIND1

// TODO: ROTARY ENCODER MAPPING
const int MIN_DISTANCE = 1;       // MINIMUM ROTARY ENCODER VALUE
const int MAX_DISTANCE = 1450;      // MAXIMUM ROTARY ENCODER VALUE
const int DISTANCE_MARGIN = MAX_DISTANCE * 0.1; // POSITION MARGIN IS 10% OF MAX DISTANCE (TBD)

//TODO: TESTING TO SET DEFUALT RETURN MOTOR SPEED
const int RETURN_SPEED = 125;     // "REASONABLY SAFE" SPEED FOR E-STOP PROCEDURES

// sampling process flags (forwards and backwards)
bool sample_done;
bool return_trip;

// emergency stop flag for interrupt handling
bool flag;

int main() {

  initLED();
  initTimer0();
  initTimer1();
  initPWMTimer3();
  initPWMTimerSpeed();
  initSwitchPD0();
  initSwitchPD1();
  initADC_A0(); // remove with full dial implementation

  Serial.begin(9600);

  // sets motor control pins to output
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  // sets rotary encoder pins as pull-up inputs
  pinMode(ROT_A, INPUT_PULLUP);
  // Set interrupt trigger for rising edge
  EICRA |= (1 << ISC31) | (1 << ISC30); // Rising edge trigger for INT3
  // Enable INT3 in the external interrupt mask register
  EIMSK |= (1 << INT3);

  sei(); // enables global interrupts

  unsigned int DIAL_SPD_MEASURE;
  unsigned int DIAL_SPD;
//  unsigned int DIAL_DIST_MEASURE;
//  unsigned int DIAL_DIST;
//  unsigned int SET_DISTANCE;

  flag = false;
  

// Debug Signals
  unsigned int counter = 0;
  int buttonPress;

  


  while(1) {
    counter = 0; // debug signal
    if (!flag) {
    switch(arm) {
        case idle: // handler for idle state
            turnOnLEDWithChar(0b100); // SOLID GREEN LED STATUS
            DIAL_SPD_MEASURE = ADCL;
            DIAL_SPD_MEASURE += ((unsigned int) ADCH) << 8;
            DIAL_SPD = map(DIAL_SPD_MEASURE, 0, 1023, 0, 255); // translates the potentiometer value to PWM range

/*-----CODE FOR WHEN ROTARY ENCODER IS IMPLEMENTED-----
            // Sets the ADC signal to the Speed-altering Dial Pin
            initADC_A0();
            // Reads the value of the potentiometer
            DIAL_SPD_MEASURE = ADCL;
            DIAL_SPD_MEASURE += ((unsigned int) ADCH) << 8;
            // translates the potentiometer value to PWM range
            DIAL_SPEED = map(DIAL_SPD_MEASURE, 0, 1023, 0, 255); 

            // Sets the ADC signal to the distance-altering Dial Pin
            initADC_A1();
            // Reads the value of the potentiometer
            DIAL_DIST_MEASURE = ADCL;
            DIAL_DIST_MEASURE += ((unsigned int) ADCH) << 8;
            // translates  potentiometer value to rotarcy encoder range
            DIAL_DIST = map (DIAL_SPD_MEASURE, 0, 1023, 0, ROTARY_MAX); // TODO: DETERMINE ROTARY ENCODER MAX VALUE
  -----------------------------------------------------*/

            // Sample Button Press handler (switches to sample state)
            buttonPress = PIND & (1 << PD1);
            if (buttonPress == 0) {  
              arm = sample;
            }
          break;
        case sample: // handler for sample state
            sample_done = false;
            turnOnLEDWithChar(0b010); // SOLID YELLOW LED STATUS
            // Loop to send sample bucket out to set distance
            while(!sample_done) {
              counter++; // debug signal
              analogWrite(L_PWM, 0);
              // Increases Speed of Motor overtime (not instant speed increase)
              for (unsigned int fastSpeed = 0; fastSpeed < DIAL_SPD; fastSpeed++) {
                analogWrite(R_PWM, fastSpeed);
                delayMs1(1);
              }
              // when the arm reaches set max distance, switch motor direction.
              if (/*ROT_VAL >= SET_DISTANCE - DISTANCE MARGIN */ counter >= 10) { 
                // Gradually slows motor speed until full stop (no immediate stops)
                for (int slowSpeed = DIAL_SPD; slowSpeed > 0; slowSpeed--) {
                  analogWrite(R_PWM, slowSpeed);
                  delayMs1(1);
                }
                delayMs0(250); // 0.25 sec delay
                // Full stop
                analogWrite(R_PWM, 0);
                analogWrite(L_PWM, 0);
                sample_done = true;
                return_trip = false;
              }
              delayMs0(1000);
            }
            counter = 0; // debug signal
            // Emergency Stop Check
            if (flag) break;



            // Loop to return sample bucket to initial position
            while(!return_trip) {
              for (unsigned int fastSpeed = 0; fastSpeed < DIAL_SPD; fastSpeed++) {
                  analogWrite(L_PWM, fastSpeed);
                  delayMs1(1);
                }
              // when sample is within a distance margin, begin slowing down.
              if (/*ROT_VAL <= MIN_DISTANCE + DISTANCE_MARGIN ||*/counter == 10) {
                for (int slowSpeed = DIAL_SPD; slowSpeed > 10; slowSpeed--) {
                  analogWrite(L_PWM, slowSpeed);
                  delayMs1(1);
                }
                // stop the motor if the sample has returned fully
                if (/*ROT_VAL < 1*/ L_PWM == 0) {
                  analogWrite(L_PWM, 0);
                  analogWrite(R_PWM, 0);
                }
                return_trip = true;
              }
              
              counter++; // debug signal
            }



            if (flag) break;
            // sets the state back to idle
            if (arm != wait) arm = idle; 
            break;
        case wait: // handler for wait state
            turnOnLEDWithChar(0b001);
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
    // If the sample is already moving back to starting position, speed up to "safe" return speed or continue at current speed
    if (L_PWM > 0) {
      for (int slowSpeed = L_PWM; slowSpeed < RETURN_SPEED; slowSpeed++) {
        analogWrite(L_PWM, slowSpeed);
      }
    // If sample is moving forwards, stop movement, and return to starting position
    } else {
      analogWrite(L_PWM, 0);
      analogWrite(R_PWM, 0);
      delayMs1(500);
      analogWrite(L_PWM, RETURN_SPEED);
    }
// Update LED's to inform user of current state
  for(int i = 0; i < 10; i++) {
/* while(ROT_VAL > 0) { */
    Serial.println(i);
    turnOnLEDWithChar(0b000); //blinking red LED (1 second cycle)
    delayMs1(500);
    turnOnLEDWithChar(0b001);
    delayMs1(500);
  }
  arm = wait;
  flag = false;
}
} 
  return 0;
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


















/* CODE ARCHIVE*/

//ISR(INT0_vect) {
//   //switches states
//   Serial.println("INTERRUPT");
//   delayMs0(1);
//   switch(arm) {
//     case sample:
//       changeDutyCycleSpeed(RETURN_SPEED); // RETURN_SPEED should be a "safe" speed to return the arm back to base
//       for(int i = 0; i < 10; i++) {
//         Serial.println(i);
//         turnOnLEDWithChar(0b000); //blinking red LED (1 second cycle)
//         delayMs1(500);
//         turnOnLEDWithChar(0b001);
//         delayMs1(500);
//         unsigned int SENSOR_DATA = analogRead(position_sensor_PIN);
//         unsigned int SENSOR_LOC = map(SENSOR_DATA, 0, 1023, 0, MAX_DISTANCE);
//         if (SENSOR_LOC == MIN_DISTANCE) {
//           changeDutyCycleSpeed(0); // stops motor
//           i = 10;
//         }
//       }
//       Serial.println("REACHED_INT");
//       arm = wait;
//     break;
//     case wait:
//       Serial.println("REACHED_WAIT");
//       arm = idle;
//     break;
//     case idle:
//       Serial.println("REACED_IDLE");
//       arm = wait;
//     break;
//     default:
//       arm = wait;
//     break;
//   }
// }





//Switch case for debounce states
  // switch(dbState) {
  // //do nothing while waiting
  //   case wait_press:
  //   break;
  // //debounce press adds delay and goes to wait_release
  //   case debounce_press:
  //   delayMs0(1);
  //   dbState = wait_release;
  //   break;
  // //Do nothing while waiting
  //   case wait_release:
  //   break;
  // //After release, delay and then go back to waiting for press
  //   case debounce_release:
  //     delayMs0(1);
  //     //switches states
  //     // if (arm == sample) {
  //     //   arm = e_stop;
  //     //   changeDutyCycleSpeed(RETURN_SPEED); // RETURN_SPEED should be a "safe" speed to return the arm back to base
  //     //   for(int i = 0; i < 10; i++) {
  //     //     turnOnLEDWithChar(0b001); //blinking red LED (1 second cycle)
  //     //     delayMs1(500);
  //     //     turnOnLEDWithChar(0b000);
  //     //     delayMs1(500);
  //     //     unsigned int SENSOR_DATA = analogRead(position_sensor_PIN);
  //     //     unsigned int SENSOR_LOC = map(SENSOR_DATA, 0, 1023, 0, MAX_DISTANCE);
  //     //     if (SENSOR_LOC == MIN_DISTANCE) {
  //     //       changeDutyCycleSpeed(0); // stops motor
  //     //       i = 10;
  //     //     }
  //     //   }
  //     //   arm = wait;
  //     //   dbState = wait_press;
  //     // } else if (arm == wait){
  //     //   arm = idle;
  //     // } else {
  //     // }
  //   break;
  // }
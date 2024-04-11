// #include <Arduino.h>
// #include <avr/io.h>
// #include "switch.h"
// #include "timer.h"
// #include "pwm.h"
// #include "led.h"
// #include "adc.h"

// /*
//   Declares states from state machine:
//     idle - waiting at base
//     sample - moving/taking sample
//     e_stop - return to base and stay until wait state left
//     wait - post e_stop idle state
// */
// typedef enum {idle, sample, e_stop, wait} states;
// states arm = idle;

// typedef enum {wait_press, debounce_press, wait_release, debounce_release} debounce;
// //define global variable for debounce states
// volatile debounce dbState = wait_press;

// //const int speed_pot_PIN = A0;     // SPEED DIAL WIRED TO AnalogIn A0
// const int distance_pot_PIN = PB6;  // DISTANCE DIAL WIRED TO PINB6

// const int motor_enable_PIN = 11;      // DIGITAL OUTPUT FOR MOTOR POWER
// const int direction_forward_PIN = 9; // DIGITAL OUTPUT FOR H-BRIDGE TO CONTROL MOTOR DIRECTION
// const int direction_reverse_PIN = 10; // DIGITAL OUTPUT FOR H-BRIDGE TO CONTROL MOTOR DIRECTION

// const int position_sensor_PIN = PA4; // ANALOG INPUT OF POSITION SENSOR DATA

// const int PIN_BUTTON = PD1;     // TAKE SAMPLE BUTTON PIND1

// // TODO: TESTING TO SET SAFETY MARGIN ON DISTANCE
// const int MIN_DISTANCE = 1;       // MINIMUM ARM DISTANCE
// const int MAX_DISTANCE = 10;      // MAXIMUM ARM DISTANCE
// const int DISTANCE_MARGIN = MAX_DISTANCE * 0.1; // POSITION MARGIN IS 10% OF MAX DISTANCE (TBD)
// //TODO: TESTING TO SET DEFUALT RETURN MOTOR SPEED
// const int RETURN_SPEED = 125;     // "REASONABLY SAFE" SPEED FOR E-STOP PROCEDURES

// bool sample_done;
// bool return_trip;

// bool flag;

// int main() {

//   initLED();
//   initTimer0();
//   initTimer1();
//   initPWMTimer3();
//   initPWMTimerSpeed();
//   initSwitchPD0();
//   initSwitchPD1();
//   initADC_A0();

//   sei(); // enables global interrupts

//   Serial.begin(9600);

//   pinMode(motor_enable_PIN, OUTPUT);
//   pinMode(direction_forward_PIN, OUTPUT);
//   pinMode(direction_reverse_PIN, OUTPUT);

//   unsigned int DIAL_SPD_MEASURE;
//   unsigned int DIAL_SPEED;
//   unsigned int DIAL_DIST_MEASURE;
//   unsigned int DIAL_DISTANCE;
//   unsigned int SET_DISTANCE;

//   unsigned int SENSOR_DATA;
//   unsigned int SENSOR_LOC;


// // Debug Signals
//   unsigned int counter = 0;
//   int buttonPress;
//   flag = false;
  


//   while(1) {
//     counter = 0;
//     //delayMs0(1000);
//     if (!flag) {
//     switch(arm) {
//         case idle: // handler for idle state
//             turnOnLEDWithChar(0b100); // SOLID GREEN LED STATUS
//             digitalWrite(motor_enable_PIN, LOW);
//             DIAL_SPD_MEASURE = ADCL;
//             DIAL_SPD_MEASURE += ((unsigned int) ADCH) << 8;
//             DIAL_SPEED = map(DIAL_SPD_MEASURE, 0, 1023, 0, 255); // translates the potentiometer value to PWM range
//             Serial.println(DIAL_SPEED);

// // TODO: TESTING TO SET SAFETY MARGIN ON MOTOR SPEED
//             // DIAL_DIST_MEASURE = analogRead(distance_pot_PIN); // see DIAL_SPD_MEASURE initialization
//             // DIAL_DISTANCE = map(DIAL_DIST_MEASURE, 0, 1023, 0, 255);
//             // SET_DISTANCE = DIAL_DISTANCE;
//             // if (SET_DISTANCE >= MAX_DISTANCE) { // Depending on the range of the dial, the distance will still not exceed the max distance
//             //   SET_DISTANCE = MAX_DISTANCE;
//             // }
//             buttonPress = PIND & (1 << PD1);
//             if (buttonPress == 0) {  // Changes states to "sample" when the button is pressed
//               arm = sample;
//             }
//           break;
//         case sample: // handler for sample state
//             sample_done = false;
//             digitalWrite(motor_enable_PIN, HIGH);
//             digitalWrite(direction_forward_PIN, HIGH); // switches direction of motor to forward
//             digitalWrite(direction_reverse_PIN, LOW);
//             turnOnLEDWithChar(0b010); // SOLID YELLOW LED STATUS
//             Serial.println("BEGIN SAMPLE");
//             while(!sample_done) {
//               counter++;
//               changeDutyCycleSpeed(DIAL_SPEED);    // MOTOR STARTS MOVING (WITH SET SPEED)
//               // SENSOR_DATA = analogRead(position_sensor_PIN);
//               // SENSOR_LOC = map(SENSOR_DATA, 0, 1023, 0, SET_DISTANCE);
//               Serial.println("TAKE SAMPLE");
//               if (SENSOR_LOC >= SET_DISTANCE - DISTANCE_MARGIN || counter >= 10) { // when the arm reaches set max distance, switch motor direction.
//                 digitalWrite(motor_enable_PIN, LOW);
//                 delayMs0(1000);
//                 changeDutyCycleSpeed(0); // Stops motor
//                 digitalWrite(direction_forward_PIN, LOW); // switches motor direction to reverse
//                 digitalWrite(direction_reverse_PIN, HIGH);
//                 changeDutyCycleSpeed(DIAL_SPEED); // MOTOR STARTS MOVING IN REVERSE DIRECTION
//                 digitalWrite(motor_enable_PIN, HIGH);
//                 sample_done = true;
//                 return_trip = false;
//               }
//               delayMs0(1000);
//             }
//             counter = 0;
//             Serial.println("TURN SAMPLE");
//             if (flag) break;
//             while(!return_trip) {
//               Serial.println("SAMPLE TURNED");
//               //SENSOR_DATA = analogRead(position_sensor_PIN);
//               //SENSOR_LOC = map(SENSOR_DATA, 0, 1023, 0, SET_DISTANCE); 
//               if (/*SENSOR_LOC <= MIN_DISTANCE + DISTANCE_MARGIN ||*/counter == 10) {
//                 changeDutyCycleSpeed(0); // stops motor with dutyCycle = 0
//                 return_trip = true;
//               }
//               counter++;
//             }
//             Serial.println("RETURN SAMPLE");
//             if (flag) break;
//             if (arm != wait) arm = idle; // sets the state back to idle
//             break;
//         case wait: // handler for wait state
//             turnOnLEDWithChar(0b001);
//             digitalWrite(motor_enable_PIN, LOW);
//             break;
//         case e_stop:
//             flag = true;
//             break;
//         default:
//             digitalWrite(motor_enable_PIN, LOW);
//             arm = idle;
//             break;
//     }
//   } else {
//   //changeDutyCycleSpeed(RETURN_SPEED); // RETURN_SPEED should be a "safe" speed to return the arm back to base
//   for(int i = 0; i < 10; i++) {
//     Serial.println(i);
//     turnOnLEDWithChar(0b000); //blinking red LED (1 second cycle)
//     delayMs1(500);
//     turnOnLEDWithChar(0b001);
//     delayMs1(500);
//     //unsigned int SENSOR_DATA = analogRead(position_sensor_PIN);
//     //unsigned int SENSOR_LOC = map(SENSOR_DATA, 0, 1023, 0, MAX_DISTANCE);
//     //if (SENSOR_LOC == MIN_DISTANCE) {
//     //   changeDutyCycleSpeed(0); // stops motor
//     //   i = 10;
//     // }
//   }
//   arm = wait;
//   flag = false;
// }
// } 
//   return 0;
// }

// // Interrupt Handler (E-Stop Button)
// ISR(INT0_vect) {
//   sample_done = true;
//   return_trip = false;
//   if (arm == wait) {
//     arm = idle;
//     flag = false;
//   } else if (arm == idle) {
//     arm = wait;
//     flag = false;
//   } else {
//     flag = true;
//     arm = wait;
//   }

//   // if (!flag) {
//   //   flag = true;
//   // } else {
//   //   flag = false;
//   // }

// }


















// /* METHOD ARCHIVE*/

// //ISR(INT0_vect) {
// //   //switches states
// //   Serial.println("INTERRUPT");
// //   delayMs0(1);
// //   switch(arm) {
// //     case sample:
// //       changeDutyCycleSpeed(RETURN_SPEED); // RETURN_SPEED should be a "safe" speed to return the arm back to base
// //       for(int i = 0; i < 10; i++) {
// //         Serial.println(i);
// //         turnOnLEDWithChar(0b000); //blinking red LED (1 second cycle)
// //         delayMs1(500);
// //         turnOnLEDWithChar(0b001);
// //         delayMs1(500);
// //         unsigned int SENSOR_DATA = analogRead(position_sensor_PIN);
// //         unsigned int SENSOR_LOC = map(SENSOR_DATA, 0, 1023, 0, MAX_DISTANCE);
// //         if (SENSOR_LOC == MIN_DISTANCE) {
// //           changeDutyCycleSpeed(0); // stops motor
// //           i = 10;
// //         }
// //       }
// //       Serial.println("REACHED_INT");
// //       arm = wait;
// //     break;
// //     case wait:
// //       Serial.println("REACHED_WAIT");
// //       arm = idle;
// //     break;
// //     case idle:
// //       Serial.println("REACED_IDLE");
// //       arm = wait;
// //     break;
// //     default:
// //       arm = wait;
// //     break;
// //   }
// // }





// //Switch case for debounce states
//   // switch(dbState) {
//   // //do nothing while waiting
//   //   case wait_press:
//   //   break;
//   // //debounce press adds delay and goes to wait_release
//   //   case debounce_press:
//   //   delayMs0(1);
//   //   dbState = wait_release;
//   //   break;
//   // //Do nothing while waiting
//   //   case wait_release:
//   //   break;
//   // //After release, delay and then go back to waiting for press
//   //   case debounce_release:
//   //     delayMs0(1);
//   //     //switches states
//   //     // if (arm == sample) {
//   //     //   arm = e_stop;
//   //     //   changeDutyCycleSpeed(RETURN_SPEED); // RETURN_SPEED should be a "safe" speed to return the arm back to base
//   //     //   for(int i = 0; i < 10; i++) {
//   //     //     turnOnLEDWithChar(0b001); //blinking red LED (1 second cycle)
//   //     //     delayMs1(500);
//   //     //     turnOnLEDWithChar(0b000);
//   //     //     delayMs1(500);
//   //     //     unsigned int SENSOR_DATA = analogRead(position_sensor_PIN);
//   //     //     unsigned int SENSOR_LOC = map(SENSOR_DATA, 0, 1023, 0, MAX_DISTANCE);
//   //     //     if (SENSOR_LOC == MIN_DISTANCE) {
//   //     //       changeDutyCycleSpeed(0); // stops motor
//   //     //       i = 10;
//   //     //     }
//   //     //   }
//   //     //   arm = wait;
//   //     //   dbState = wait_press;
//   //     // } else if (arm == wait){
//   //     //   arm = idle;
//   //     // } else {
//   //     // }
//   //   break;
//   // }
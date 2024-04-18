# ENGR498_24070_Arduino

Software to program an ATMega2560 Arduino board to drive our automated material sampling system. The source file contains a main.cpp for the general system as well as a moduleTest.cpp for testing individual hardware components such as the DC motor or Rotary Encoder. Additional files include register and pin initializations for PWM sigals, ADC processes, as well as timers and LED activation. (The files are named respectively to their .h, .cpp pairs). This code was developed by Sam Kerns with synchronous discussion with Nicholas Hinrichs.

List of Hardware Components implemented in main:
  1. 1 DC Motor
  2. 1 2-phase Rotary Encoder
  3. 3 LED's
  4. 2 10K potentiometers
  5. 2 buttons
  6. 1 BTS7960 H-Bridge IC

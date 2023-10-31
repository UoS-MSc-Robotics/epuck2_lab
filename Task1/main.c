//
// MIT License
//
// Copyright (c) 2023 Shakir Abdul Rasheed
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <ch.h>
#include <hal.h>
#include <memory_protection.h>
#include <main.h>

// LEDS
#include <leds.h>
#include <spi_comm.h>
// PROXIMITY
#include <sensors/proximity.h>
// TOF
#include <sensors/VL53L0X/VL53L0X.h>
// BLUETOOTH
#include <epuck1x/uart/e_uart_char.h>
#include <serial_comm.h>
// MOTORS
#include <motors.h>
// SELECTOR
#include <selector.h>


// Macros
#define MAX_SPEED 500
#define THRESHOLD 75
#define CRITICAL_THRESHOLD 150

// Tuning
double braitenberg_weights[3] = {0.8, 0.6, 0.6}; // sum should be less than 2

// Initialization
double proximity_values[8];  // sensor readings
double proximity_weights[8]; // collection of ones and zeroes
double left_speed = 0.0;
double right_speed = 0.0;

// Proximity sensors initialization
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*
 * Function run the initialization process
*/
void init() {
  halInit();
  chSysInit();
  mpu_init();

  // LED Initialization
  clear_leds();
  spi_comm_start();

  // Motor Initialization
  motors_init();

  // Selector Initialization
  serial_start();

  // Proximity Initialization
  messagebus_init(&bus, &bus_lock, &bus_condvar);
  proximity_start(0);
  calibrate_ir();

  // TOF Initialization
  VL53L0X_start();
}

/*
 * Function to fill sensor values
*/
void fill_sensor_values() {
  for (int i = 0; i < 8; i++) {
    proximity_values[i] = get_calibrated_prox(i);
  }
}

/*
 * Function to fill proximity weights
*/
void fill_proximity_weights() {
  for (int i = 0; i < 8; i++) {
    if (proximity_values[i] > THRESHOLD) {
      proximity_weights[i] = 1.0;
    } else {
      proximity_weights[i] = 0.0;
    }
  }
}

/*
 * Function to stop the robot
*/
void stop() {
  left_motor_set_speed(0);
  right_motor_set_speed(0);
}

/*
 * Function to run braitenberg algorithm
*/
void run_braitenberg() {
  left_speed = 1.0 -
    (proximity_weights[0] * braitenberg_weights[0] +
     proximity_weights[1] * braitenberg_weights[1] +
     proximity_weights[2] * braitenberg_weights[2]);
  right_speed = 1.0 -
    (proximity_weights[7] * braitenberg_weights[0] +
     proximity_weights[6] * braitenberg_weights[1] +
     proximity_weights[5] * braitenberg_weights[2]);

  left_speed *= MAX_SPEED;
  right_speed *= MAX_SPEED;

  left_motor_set_speed(left_speed);
  right_motor_set_speed(right_speed);
}

/*
 * Function to rotate if critical threshold is reached only if one sensor detects
*/
bool stray_criticals() {
  // 5, 6, 7
  if (((proximity_values[5] > CRITICAL_THRESHOLD) && !(proximity_weights[6]) && !(proximity_weights[7]) &&
       !(proximity_weights[0]) && !(proximity_weights[1]) && !(proximity_weights[2])) ||

      (!(proximity_weights[5]) && (proximity_values[6] > CRITICAL_THRESHOLD) && !(proximity_weights[7]) &&
        !(proximity_weights[0]) && !(proximity_weights[1]) && !(proximity_weights[2])) ||

        (!(proximity_weights[5]) && !(proximity_weights[6]) && (proximity_values[7] > CRITICAL_THRESHOLD) &&
        !(proximity_weights[0]) && !(proximity_weights[1]) && !(proximity_weights[2]))) {

    // stop
    stop();
    chThdSleepMilliseconds(500);

    // rotate right
    left_speed = MAX_SPEED/2;
    right_speed = -MAX_SPEED/2;

    left_motor_set_speed(left_speed);
    right_motor_set_speed(right_speed);
    chThdSleepMilliseconds(500);

    return true;

  // 0, 1, 2
  } else if (((proximity_values[0] > CRITICAL_THRESHOLD) && !(proximity_weights[1]) && !(proximity_weights[2]) &&
              !(proximity_weights[5]) && !(proximity_weights[6]) && !(proximity_weights[7])) ||

             (!(proximity_weights[0]) && (proximity_values[1] > CRITICAL_THRESHOLD) && !(proximity_weights[2]) &&
              !(proximity_weights[5]) && !(proximity_weights[6]) && !(proximity_weights[7])) ||

             (!(proximity_weights[0]) && !(proximity_weights[1]) && (proximity_values[2] > CRITICAL_THRESHOLD) &&
              !(proximity_weights[5]) && !(proximity_weights[6]) && !(proximity_weights[7]))) {

    // stop
    stop();
    chThdSleepMilliseconds(500);

    // rotate right
    left_speed = -MAX_SPEED/2;
    right_speed = MAX_SPEED/2;

    left_motor_set_speed(left_speed);
    right_motor_set_speed(right_speed);
    chThdSleepMilliseconds(500);

    return true;

 } else {
    return false;
  }
}


int main(void) {

  init();

  /* Infinite loop. */
  while (1) {
    fill_sensor_values();
    fill_proximity_weights();

    // check for stray criticals
    if (stray_criticals()) {
      continue;
    }

    // U Block
    if ((proximity_weights[5] || proximity_weights[6]) &&
        (proximity_weights[1] || proximity_weights[2]) &&
        (proximity_weights[0] || proximity_weights[7])) {

        left_speed = -MAX_SPEED;
        right_speed = MAX_SPEED;

        left_motor_set_speed(left_speed);
        right_motor_set_speed(right_speed);
        chThdSleepMilliseconds(500);

    } else {
        run_braitenberg();
    }

    // set all sensor weights to 0
    for (int i = 0; i < 8; i++) {
      proximity_weights[i] = 0.0;
    }

    // slow down update rate
    chThdSleepMilliseconds(100);
  }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

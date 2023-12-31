//
// MIT License
//
// Copyright (c) 2023 Leander Stephen Desouza
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
#define MAX_SPEED 650
#define P_THRESHOLD 100
#define CRITICAL_THRESHOLD 250
#define START_SELECTOR 0

// Tuning
double braitenberg_weights[3] = {0.7, 0.5, 0.3}; // sum should be less than 2

// Initialization
double proximity_values[8];  // sensor readings
double proximity_weights[8]; // collection of ones and zeroes
double left_speed = 0.0;
double right_speed = 0.0;
double tof_value;

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

void send_bt_values() {
  // send calibrated proximity values through bluetooth
  char prox_str[100];
  int str_length;
  str_length = sprintf(prox_str, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,\n",
    get_calibrated_prox(0),
    get_calibrated_prox(1),
    get_calibrated_prox(2),
    get_calibrated_prox(3),
    get_calibrated_prox(4),
    get_calibrated_prox(5),
    get_calibrated_prox(6),
    get_calibrated_prox(7),
    tof_value,
    left_motor_get_desired_speed(),
    right_motor_get_desired_speed(),
    get_selector()
  );
  e_send_uart1_char(prox_str, str_length);    // obstacle avoidance
}

/*
 * Function to fill sensor values
*/
void fill_sensor_values() {
  for (int i = 0; i < 8; i++) {
    proximity_values[i] = get_calibrated_prox(i);
  }
  tof_value = VL53L0X_get_dist_mm();
}

/*
 * Function to fill proximity weights
*/
void fill_proximity_weights() {
  for (int i = 0; i < 8; i++) {
    if (proximity_values[i] > P_THRESHOLD) {
      proximity_weights[i] = 1.0;
    } else {
      proximity_weights[i] = 0.0;
    }
  }
}

/*
 * Function to glow LEDs
*/
void glow_leds() {
  if((get_calibrated_prox(0) > P_THRESHOLD) || (get_calibrated_prox(7) > P_THRESHOLD)) {
    e_set_led(0, 1);
  } else {
    e_set_led(0, 0);
  }

  if(get_calibrated_prox(1) > P_THRESHOLD) {
    e_set_led(1, 1);
  } else {
    e_set_led(1, 0);
  }

  if(get_calibrated_prox(2) > P_THRESHOLD) {
    e_set_led(2, 1);
  } else {
    e_set_led(2, 0);
  }

  if(get_calibrated_prox(3) > P_THRESHOLD) {
    e_set_led(3, 1);
  } else {
    e_set_led(3, 0);
  }

  if((get_calibrated_prox(3) > P_THRESHOLD) || (get_calibrated_prox(4) > P_THRESHOLD)) {
    e_set_led(4, 1);
  } else {
    e_set_led(4, 0);
  }

  if(get_calibrated_prox(4) > P_THRESHOLD) {
    e_set_led(5, 1);
  } else {
    e_set_led(5, 0);
  }

  if(get_calibrated_prox(5) > P_THRESHOLD) {
    e_set_led(6, 1);
  } else {
    e_set_led(6, 0);
  }

  if(get_calibrated_prox(6) > P_THRESHOLD) {
    e_set_led(7, 1);
  } else {
    e_set_led(7, 0);
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

    if (get_selector() != START_SELECTOR) {
      stop();
      chThdSleepMilliseconds(100);
      continue;
    }

    glow_leds();
    send_bt_values();
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
    chThdSleepMilliseconds(50);
  }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}

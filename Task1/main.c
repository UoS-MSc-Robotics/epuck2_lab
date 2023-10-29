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

#include <leds.h>
#include <spi_comm.h>
#include <sensors/proximity.h>
#include <sensors/VL53L0X/VL53L0X.h>
#include <epuck1x/uart/e_uart_char.h>
#include <stdio.h>
#include <serial_comm.h>
#include <motors.h>
#include <selector.h>


#define MAX_SPEED 500
#define THRESHOLD 90.0


double leftSpeed = 0.0;
double rightSpeed = 0.0;

double sensorValues[8] =
  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double sensorWeight[8] =
  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double proximityWeights[3] = {0.8, 0.6, 0.3}; // [ps0, ps1, ps2] && [ps7, ps6, ps5]

// proximity sensors initialization
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

/*
 * Function to find the largest free gap
*/
void findLargestGapCenter() {
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > THRESHOLD) {
      sensorWeight[i]++;
    }
  }
}

/*
 * Function to get sensor values
*/
void get_sensor_values() {
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = get_calibrated_prox(i);
  }
}

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


int main(void) {

  init();

  /* Infinite loop. */
  while (1) {

    leftSpeed = 0.8 * MAX_SPEED;
    rightSpeed = 0.8 * MAX_SPEED;

    get_sensor_values();
    findLargestGapCenter();

    if (sensorWeight[1] > 0 && sensorWeight[6] > 0) {

      if (sensorWeight[3] > 0) {
        leftSpeed = MAX_SPEED / 10;
        rightSpeed = -MAX_SPEED / 10;

      } else if (sensorWeight[4] > 0) {
        leftSpeed = -MAX_SPEED / 10;
        rightSpeed = MAX_SPEED / 10;

      } else {
        leftSpeed = -MAX_SPEED;
        rightSpeed = MAX_SPEED;
      }

    } else {
        leftSpeed = 1.0 - (proximityWeights[0] * sensorWeight[0] +
                           proximityWeights[1] * sensorWeight[1] +
                           proximityWeights[2] * sensorWeight[2]);
        rightSpeed = 1.0 - (proximityWeights[0] * sensorWeight[7] +
                            proximityWeights[1] * sensorWeight[6] +
                            proximityWeights[2] * sensorWeight[5]);

        leftSpeed *= MAX_SPEED;
        rightSpeed *= MAX_SPEED;
    }

    // set all sensor weights to 0
    for (int i = 0; i < 8; i++) {
      sensorWeight[i] = 0;
    }

    // set motor speeds
    left_motor_set_speed(leftSpeed);
    right_motor_set_speed(rightSpeed);

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

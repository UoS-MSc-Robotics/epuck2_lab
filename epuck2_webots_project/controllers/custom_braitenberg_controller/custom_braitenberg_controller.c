//
// MIT License
//
// Copyright (c) 2023 Leander Stephen Desouza

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
#include <string.h>
#include <stdlib.h>

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>


// Macros
#define MAX_SPEED 6.27/2
#define THRESHOLD 75
#define CRITICAL_THRESHOLD 150
#define TIME_STEP 64

// Tuning
double braitenberg_weights[3] = {0.7, 0.5, 0.3}; // sum should be less than 2

// Initialization
double proximity_values[8];  // sensor readings
double proximity_weights[8]; // collection of ones and zeroes
double left_speed = 0.0;
double right_speed = 0.0;

// Webots stuff
const char *proximity_sensor_names[8] = {
  "ps0", "ps1", "ps2", "ps3",
  "ps4", "ps5", "ps6", "ps7"
};

WbDeviceTag left_motor;
WbDeviceTag right_motor;
WbDeviceTag proximity_sensor[8];

/*
 * Webots stuff for delay
*/

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(0);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

/*
 * Function run the initialization process
*/
void init() {
  wb_robot_init();

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 1.0);
  wb_motor_set_velocity(right_motor, 1.0);

  for (int i = 0; i < 8; i++) {
    proximity_sensor[i] = wb_robot_get_device(proximity_sensor_names[i]);
    wb_distance_sensor_enable(proximity_sensor[i], TIME_STEP);
  }
}

/*
 * Function to fill sensor values
*/
void fill_sensor_values() {
  for (int i = 0; i < 8; i++) {
    proximity_values[i] = wb_distance_sensor_get_value(proximity_sensor[i]);
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
  // printf("Proximity Weights: %f %f %f %f %f %f %f %f\n",
  //        proximity_weights[0], proximity_weights[1], proximity_weights[2],
  //        proximity_weights[3], proximity_weights[4], proximity_weights[5],
  //        proximity_weights[6], proximity_weights[7]);
}

/*
 * Function to stop the robot
*/
void stop() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

/*
 * Function to run braitenberg algorithm
*/
void run_braitenberg() {
  printf("Braitenberg\n");
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

  wb_motor_set_velocity(left_motor, left_speed);
  wb_motor_set_velocity(right_motor, right_speed);
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

    // rotate right
    printf("left crit\n");
    left_speed = MAX_SPEED/2;
    right_speed = -MAX_SPEED/2;

    stop();
    passive_wait(0.5);
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
    passive_wait(0.5);
    return true;

  // 0, 1, 2
  } else if (((proximity_values[0] > CRITICAL_THRESHOLD) && !(proximity_weights[1]) && !(proximity_weights[2]) &&
              !(proximity_weights[5]) && !(proximity_weights[6]) && !(proximity_weights[7])) ||

             (!(proximity_weights[0]) && (proximity_values[1] > CRITICAL_THRESHOLD) && !(proximity_weights[2]) &&
              !(proximity_weights[5]) && !(proximity_weights[6]) && !(proximity_weights[7])) ||

             (!(proximity_weights[0]) && !(proximity_weights[1]) && (proximity_values[2] > CRITICAL_THRESHOLD) &&
              !(proximity_weights[5]) && !(proximity_weights[6]) && !(proximity_weights[7]))) {

    // rotate left
    printf("right crit\n");
    left_speed = -MAX_SPEED/2;
    right_speed = MAX_SPEED/2;

    stop();
    passive_wait(0.5);
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
    passive_wait(0.5);
    return true;

 } else {
    return false;
  }
}


int main(int argc, char **argv) {
  init();


  while (wb_robot_step(TIME_STEP) != -1) {
    fill_sensor_values();
    fill_proximity_weights();

    if (stray_criticals()) {
      continue;
    }


    if ((proximity_weights[5] || proximity_weights[6]) &&
        (proximity_weights[1] || proximity_weights[2]) &&
        (proximity_weights[0] || proximity_weights[7])) {

        printf("U Block\n");

        left_speed = -MAX_SPEED/2;
        right_speed = MAX_SPEED/2;

        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
        passive_wait(0.5);

    } else {
        run_braitenberg();
    }

    // set all sensor weights to 0
    for (int i = 0; i < 8; i++) {
      proximity_weights[i] = 0.0;
    }

    passive_wait(0.1);

  };
  wb_robot_cleanup();

  return 0;
}

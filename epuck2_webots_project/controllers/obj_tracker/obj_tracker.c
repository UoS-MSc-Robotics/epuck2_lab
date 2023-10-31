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
#define P_THRESHOLD 75
#define T_THRESHOLD 300
#define CRITICAL_THRESHOLD 150
#define TIME_STEP 64

// Tuning
double braitenberg_weights[3] = {0.3, 0.7, 1.0}; // sum should be less than 2

// Initialization
double proximity_values[8];  // sensor readings
double proximity_weights[8]; // collection of ones and zeroes
double tof_value;            // tof sensor reading
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
WbDeviceTag tof_sensor;

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
  tof_sensor = wb_robot_get_device("tof");
  wb_distance_sensor_enable(tof_sensor, TIME_STEP);
}

/*
 * Function to get sensor values
*/
void get_sensor_values() {
  for (int i = 0; i < 8; i++) {
    proximity_values[i] = wb_distance_sensor_get_value(proximity_sensor[i]);
  }
  tof_value = wb_distance_sensor_get_value(tof_sensor);
  printf("tof: %f\n", tof_value);
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
 * Function to move the robot backwards
*/
void backwards(float speed) {
  wb_motor_set_velocity(left_motor, -speed);
  wb_motor_set_velocity(right_motor, -speed);
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

  wb_motor_set_velocity(left_motor, right_speed);
  wb_motor_set_velocity(right_motor, left_speed);
}



int main(int argc, char **argv) {
  init();


  while (wb_robot_step(TIME_STEP) != -1) {
    get_sensor_values();
    fill_proximity_weights();


    if (proximity_values[0] > CRITICAL_THRESHOLD &&
        proximity_values[7] > CRITICAL_THRESHOLD) {
      printf("Move backwards\n");

      // move backwards
      backwards(MAX_SPEED/2);
      continue;
    }

    // if tof sensor is not detecting anything, rotate
    if (tof_value > T_THRESHOLD &&
      !(proximity_weights[5] || proximity_weights[6] || proximity_weights[7] ||
        proximity_weights[0] || proximity_weights[1] || proximity_weights[2])) {
      printf("Rotate left\n");

      left_speed = -MAX_SPEED/2;
      right_speed = MAX_SPEED/2;

      wb_motor_set_velocity(left_motor, left_speed);
      wb_motor_set_velocity(right_motor, right_speed);

    } else {

      if ((proximity_weights[0] && proximity_weights[7])) {
        printf("Reached\n");
        stop();

      } else {
        run_braitenberg();
      }
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

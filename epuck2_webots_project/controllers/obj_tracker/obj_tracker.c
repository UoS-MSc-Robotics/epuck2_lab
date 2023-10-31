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
#define P_THRESHOLD 100
#define T_THRESHOLD 300
#define BACKWARD_THRESHOLD 250
#define FORWARD_THRESHOLD 150
#define TIME_STEP 64


// Initialization
double proximity_values[8];  // sensor readings
double proximity_weights[8]; // collection of ones and zeroes
double tof_value;            // tof sensor reading
double left_speed = 0.0;
double right_speed = 0.0;
int left_dir_counter = 0;
int right_dir_counter = 0;

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

  printf("proximity: %f %f %f %f %f %f %f %f\n",
    proximity_values[0], proximity_values[1], proximity_values[2], proximity_values[3],
    proximity_values[4], proximity_values[5], proximity_values[6], proximity_values[7]);

  tof_value = wb_distance_sensor_get_value(tof_sensor);
  // printf("tof: %f\n", tof_value);
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
  // printf("proximity weights: %f %f %f %f %f %f %f %f\n",
  //   proximity_weights[0], proximity_weights[1], proximity_weights[2], proximity_weights[3],
  //   proximity_weights[4], proximity_weights[5], proximity_weights[6], proximity_weights[7]);
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
void move_backward(float speed) {
  wb_motor_set_velocity(left_motor, -speed);
  wb_motor_set_velocity(right_motor, -speed);
}

/*
 * Function to move the robot forward
*/
void move_forward(float speed) {
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, speed);
}

/*
 * Function to turn the robot left
*/
void turn_left(float speed) {
  wb_motor_set_velocity(left_motor, -speed);
  wb_motor_set_velocity(right_motor, speed);
}

/*
 * Function to turn the robot right
*/
void turn_right(float speed) {
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, -speed);
}

/*
 * Function to get the last sensor input direction
*/
void get_last_sensor_input_direction() {
  // set counter to 0
  left_dir_counter = 0;
  right_dir_counter = 0;

  if (proximity_weights[4] || proximity_weights[5] ||
    proximity_weights[6] || proximity_weights[7]) {
    left_dir_counter++;
  } else {
    right_dir_counter++;
  }
}

int main(int argc, char **argv) {
  init();


  while (wb_robot_step(TIME_STEP) != -1) {
    get_sensor_values();
    fill_proximity_weights();

    if (proximity_values[0] > BACKWARD_THRESHOLD || proximity_values[7] > BACKWARD_THRESHOLD ||
      proximity_values[1] > BACKWARD_THRESHOLD || proximity_values[6] > BACKWARD_THRESHOLD) {

      get_last_sensor_input_direction();
      printf("Move backwards\n");
      move_backward(MAX_SPEED/2);

    } else {

        // if tof and no proximity, rotate left till object is detected
        if (tof_value > T_THRESHOLD &&
          !(proximity_weights[5] || proximity_weights[6] || proximity_weights[7] ||
            proximity_weights[0] || proximity_weights[1] || proximity_weights[2])) {
          printf("Lost connection\n");

          if (left_dir_counter > right_dir_counter) {
            printf("Rotate left\n");
            turn_left(MAX_SPEED/2);
          } else {
            printf("Rotate right\n");
            turn_right(MAX_SPEED/2);
          }

        } else if (proximity_values[0] > FORWARD_THRESHOLD || proximity_values[7] > FORWARD_THRESHOLD && tof_value < T_THRESHOLD) {

            get_last_sensor_input_direction();
            printf("Reached\n");
            stop();

        } else if (proximity_weights[5] || proximity_weights[6]) {
          get_last_sensor_input_direction();
          // rotate left
          printf("Rotate left\n");
          turn_left(MAX_SPEED/2);

        } else if (proximity_weights[1] || proximity_weights[2]) {
          get_last_sensor_input_direction();
          // rotate right
          printf("Rotate right\n");
          turn_right(MAX_SPEED/2);

        } else if (proximity_weights[7] && !proximity_weights[0] && tof_value > T_THRESHOLD) {
          get_last_sensor_input_direction();
          // move left
          printf("Move left\n");
          turn_left(MAX_SPEED/2);

        } else if (proximity_weights[0] && !proximity_weights[7] && tof_value > T_THRESHOLD) {
          get_last_sensor_input_direction();
          // move right
          printf("Move right\n");
          turn_right(MAX_SPEED/2);

        } else {
          get_last_sensor_input_direction();
          // move forward
          printf("Move forward\n");
          move_forward(MAX_SPEED/2);
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

//
// MIT License
//
// Copyright (c) 2023 Leander Stephen D'Souza
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

#include <time.h>
#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/led.h>
#include <webots/distance_sensor.h>

#define TIME_STEP 64
#define MAX_SPEED 6.27
#define P_THRESHOLD 75.0 // lower the value, higher the sensitivity
#define T_THRESHOLD 100.0 // higher the value, higher the sensitivity

int left_counter = 0;
int right_counter = 0;


WbDeviceTag left_motor;
WbDeviceTag right_motor;
WbDeviceTag proximity_sensor[8];
WbDeviceTag tof_sensor;
WbDeviceTag leds[8];

float val_proximity[8];
float tof_readings;

int sum_right;
int sum_left;
int times_left = 0;
int times_right = 0;

const char *leds_names[8] = {
  "led0", "led1", "led2", "led3",
  "led4", "led5", "led6", "led7"};

const char *proximity_sensor_names[8] = {
  "ps0", "ps1", "ps2", "ps3",
  "ps4", "ps5", "ps6", "ps7"
};

static int get_time_step() {
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

static void step() {
  if (wb_robot_step(get_time_step()) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

void init() {
  wb_robot_init();

  srand(time(NULL));

  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  tof_sensor = wb_robot_get_device("tof");
  wb_distance_sensor_enable(tof_sensor, TIME_STEP);

  for (int i = 0; i < 8; i++) {
    proximity_sensor[i] = wb_robot_get_device(proximity_sensor_names[i]);
    wb_distance_sensor_enable(proximity_sensor[i], TIME_STEP);
  }

  for (int i = 0; i < 8; i++) {
    leds[i] = wb_robot_get_device(leds_names[i]);
  }

}

void move_forward(float speed) {
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, speed);
}

void turn_left(float speed) {
  wb_motor_set_velocity(left_motor, -speed);
  wb_motor_set_velocity(right_motor, speed);
}

void turn_right(float speed) {
  wb_motor_set_velocity(left_motor, speed);
  wb_motor_set_velocity(right_motor, -speed);
}

void stop() {
  wb_motor_set_velocity(left_motor, 0);
  wb_motor_set_velocity(right_motor, 0);
}

void get_sensor_values() {
  for (int i = 0; i < 8; i++) {
    val_proximity[i] = wb_distance_sensor_get_value(proximity_sensor[i]);
    // printf("Proximity Sensor %d: %f ", i, val_proximity[i]);
  }
  tof_readings = wb_distance_sensor_get_value(tof_sensor);
  // printf("TOF Sensor: %f\n", tof_readings);
}


void obs_avoidance() {
  // left indices - 5, 6,
  // right indices - 1, 2,
  // front indices - 0, 7
  // back indices - 3, 4
  sum_left = 0.0;
  sum_right = 0.0;

  sum_right = val_proximity[0] + val_proximity[1]; // 0,1 are on right
  sum_left = val_proximity[6] + val_proximity[7]; // 7,6 are on the left

  // Check for a dead end and take appropriate action
  if ((val_proximity[0] > 500 || val_proximity[7] > 500) && val_proximity[2] > 400 && val_proximity[5] > 400) {
    printf("Dead end detected\n");
    turn_left(MAX_SPEED/3);
    passive_wait(2.0);
  }

  // Avoid running in circles
  if (times_left >= 5 || times_right >= 5) {
      times_left = 0;
      times_right = 0;
      turn_left(MAX_SPEED/3);
      passive_wait(2.0);
  }

  // Decide to run forward or turn based on proximity sensor readings
  if (val_proximity[0] <= 500 && val_proximity[1] <= 500 && val_proximity[6] <= 500 && val_proximity[7] <= 500) {
      // No obstacles in front, run forward
      move_forward(MAX_SPEED/2);
  } else {
      // Obstacle in front, make a decision to turn left or right
      if (sum_right >= sum_left) {
          // Turning left situation
          // add left audio
          times_left++;
          times_right = 0;

          turn_left(MAX_SPEED/3);
          passive_wait(0.05);

          while (val_proximity[1] > 500) {
            turn_left(MAX_SPEED/3);
            passive_wait(0.05);
          }
      } else {
          // Turning right situation
          times_right++;
          times_left = 0;
          turn_right(MAX_SPEED/3);
          passive_wait(0.05);
          while (val_proximity[6] > 500) {
            turn_right(MAX_SPEED/3);
            passive_wait(0.05);
          }
      }
  }

}


int main(int argc, char **argv) {
  init();

  while (wb_robot_step(TIME_STEP) != -1) {
    get_sensor_values();

    obs_avoidance();
    passive_wait(0.1);
  }

  wb_robot_cleanup();

  return 0;
}

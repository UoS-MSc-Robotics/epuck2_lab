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
#include <webots/distance_sensor.h>

#define TIME_STEP 64
#define MAX_SPEED 6.27
#define THRESHOLD 75.0


WbDeviceTag left_motor;
WbDeviceTag right_motor;
WbDeviceTag proximity_sensor[8];

float proximity_readings[8];

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

  for (int i = 0; i < 8; i++) {
    proximity_sensor[i] = wb_robot_get_device(proximity_sensor_names[i]);
    wb_distance_sensor_enable(proximity_sensor[i], TIME_STEP);
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

void get_sensor_values() {
  for (int i = 0; i < 8; i++) {
    proximity_readings[i] = wb_distance_sensor_get_value(proximity_sensor[i]);
    // printf("Proximity Sensor %d: %f ", i, proximity_readings[i]);
  }
  // printf("\n");
}

void obs_avoidance() {
  // left indices - 5, 6,
  // right indices - 1, 2,
  // front indices - 0, 7
  // back indices - 3, 4

  /*
    1, left side is blocked, right side is blocked, front is blocked - turn left for 1 second
    2, left side is blocked, right side is blocked, front is not blocked - move forward
    3, left side is blocked, right side is not blocked - turn right
    4, left side is not blocked, right side is blocked - turn left
    5, front is blocked, left side is not blocked, right side is not blocked - turn left
  */

  if (proximity_readings[5] > THRESHOLD && proximity_readings[6] > THRESHOLD &&
      proximity_readings[1] > THRESHOLD && proximity_readings[2] > THRESHOLD &&
      proximity_readings[0] > THRESHOLD && proximity_readings[7] > THRESHOLD) {
    printf("U-Block\n");

    // randomly choose left or right
    if (rand() % 2) {
      turn_left(MAX_SPEED);
    } else {
      turn_right(MAX_SPEED);
    }

    passive_wait(1.0); // turn for 1 second
  } else if (proximity_readings[5] > THRESHOLD && proximity_readings[6] > THRESHOLD &&
             proximity_readings[1] > THRESHOLD && proximity_readings[2] > THRESHOLD &&
             proximity_readings[0] < THRESHOLD && proximity_readings[7] < THRESHOLD) {
    printf("LR-Block\n");
    move_forward(MAX_SPEED/2);
  } else if (proximity_readings[5] > THRESHOLD && proximity_readings[6] > THRESHOLD) {
    printf("L-Block\n");
    turn_right(MAX_SPEED);
    passive_wait(0.5);
  } else if (proximity_readings[1] > THRESHOLD && proximity_readings[2] > THRESHOLD) {
    printf("R-Block\n");
    turn_left(MAX_SPEED);
    passive_wait(0.5);
  } else if (proximity_readings[0] > THRESHOLD && proximity_readings[7] > THRESHOLD) {
    printf("Headon\n");

    // randomly choose left or right
    if (rand() % 2) {
      turn_left(MAX_SPEED);
    } else {
      turn_right(MAX_SPEED);
    }
    passive_wait(1.0); // turn for 1 second

  } else if (proximity_readings[0] > THRESHOLD) {
    printf("FR-Block\n");
    turn_left(MAX_SPEED);
  } else if (proximity_readings[7] > THRESHOLD) {
    printf("FL-Block\n");
    turn_right(MAX_SPEED);
  } else {
    printf("no obstacles\n");
    move_forward(MAX_SPEED/2);
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

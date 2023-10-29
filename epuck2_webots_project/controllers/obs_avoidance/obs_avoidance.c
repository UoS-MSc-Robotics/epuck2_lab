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
#define P_THRESHOLD 75.0 // lower the value, higher the sensitivity
#define T_THRESHOLD 100.0 // higher the value, higher the sensitivity

int left_counter = 0;
int right_counter = 0;


WbDeviceTag left_motor;
WbDeviceTag right_motor;
WbDeviceTag proximity_sensor[8];
WbDeviceTag tof_sensor;
WbDeviceTag leds[8];

float proximity_readings[8];
float tof_readings;

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
    proximity_readings[i] = wb_distance_sensor_get_value(proximity_sensor[i]);
    // printf("Proximity Sensor %d: %f ", i, proximity_readings[i]);
  }
  tof_readings = wb_distance_sensor_get_value(tof_sensor);
  // printf("TOF Sensor: %f\n", tof_readings);
}



void glow_leds() {
  // glow led green if there are no obstacles, else glow red
  if (tof_readings < T_THRESHOLD ||
      proximity_readings[0] > P_THRESHOLD ||
      proximity_readings[7] > P_THRESHOLD) {
    // turn irl led1 red (epuck - led0)
    wb_led_set(leds[0], 1);
  } else {
    // turn off
    wb_led_set(leds[0], 0);
  }

  if (proximity_readings[1] > P_THRESHOLD) {
    // turn irl led2 red (epuck - led1)
    wb_led_set(leds[1], 0xff0000);
  } else {
    // turn off
    wb_led_set(leds[1], 0x000000);
  }

  if (proximity_readings[2] > P_THRESHOLD) {
    // turn irl led3 red (epuck - led2)
    wb_led_set(leds[2], 1);
  } else {
    // turn off
    wb_led_set(leds[2], 0);
  }

  if (proximity_readings[3] > P_THRESHOLD) {
    // turn irl led4 red (epuck - led3)
    wb_led_set(leds[3], 0xff0000);
  } else {
    // turn off
    wb_led_set(leds[3], 0x000000);
  }

  if (proximity_readings[4] > P_THRESHOLD) {
    // turn irl led6 red (epuck - led5)
    wb_led_set(leds[5], 0xff0000);
  } else {
    // turn off
    wb_led_set(leds[5], 0x000000);
  }

  if (proximity_readings[5] > P_THRESHOLD) {
    // turn irl led7 red (epuck - led6)
    wb_led_set(leds[6], 1);
  }  else {
    // turn off
    wb_led_set(leds[6], 0);
  }

  if (proximity_readings[6] > P_THRESHOLD) {
    // turn irl led8 red (epuck - led7)
    wb_led_set(leds[7], 0xff0000);
  } else {
    // turn off
    wb_led_set(leds[7], 0x000000);
  }

  if (tof_readings < T_THRESHOLD &&
      proximity_readings[0] > P_THRESHOLD &&
      proximity_readings[7] > P_THRESHOLD &&
      proximity_readings[1] > P_THRESHOLD &&
      proximity_readings[2] > P_THRESHOLD &&
      proximity_readings[5] > P_THRESHOLD &&
      proximity_readings[6] > P_THRESHOLD) {
    // turn irl led5 green (epuck - led4)
    wb_led_set(leds[1], 0x00ff00);
    wb_led_set(leds[3], 0x00ff00);
    wb_led_set(leds[5], 0x00ff00);
    wb_led_set(leds[7], 0x00ff00);
  } else {
    wb_led_set(leds[1], 0x000000);
    wb_led_set(leds[3], 0x000000);
    wb_led_set(leds[5], 0x000000);
    wb_led_set(leds[7], 0x000000);
  }

}


void recovery() {
  printf("Recovery\n");

  bool direction = rand() % 2;

  passive_wait(1.0);

  // turn till tof sensor is clear
  while (tof_readings < 3*T_THRESHOLD ||
         proximity_readings[0] > P_THRESHOLD ||
         proximity_readings[7] > P_THRESHOLD) {

    printf("tof: %f\n", tof_readings);

    if (direction) {
      turn_left(MAX_SPEED/2);
    } else {
      turn_right(MAX_SPEED/2);
    }

    passive_wait(0.2);
    get_sensor_values();
  }
  stop();
  passive_wait(0.5);
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

  if ((proximity_readings[5] > P_THRESHOLD || proximity_readings[6] > P_THRESHOLD) &&
      (proximity_readings[1] > P_THRESHOLD || proximity_readings[2] > P_THRESHOLD) &&
      (proximity_readings[0] > P_THRESHOLD || proximity_readings[7] > P_THRESHOLD) &&
      tof_readings < T_THRESHOLD) {
    printf("U-Block\n");
    recovery();

  } else if ((proximity_readings[5] > P_THRESHOLD || proximity_readings[6] > P_THRESHOLD) &&
      (proximity_readings[1] > P_THRESHOLD || proximity_readings[2] > P_THRESHOLD) &&
      (proximity_readings[0] < P_THRESHOLD || proximity_readings[7] < P_THRESHOLD) &&
      tof_readings > T_THRESHOLD) {
    printf("LR-Block\n");
    move_forward(MAX_SPEED/2);

  } else if (proximity_readings[5] > P_THRESHOLD || proximity_readings[6] > P_THRESHOLD) {
    printf("plain L-Block\n");
    right_counter++;
    turn_right(MAX_SPEED/2);

  } else if (proximity_readings[1] > P_THRESHOLD || proximity_readings[2] > P_THRESHOLD) {
    printf("plain R-Block\n");
    left_counter++;
    turn_left(MAX_SPEED/2);

  } else if (proximity_readings[0] > P_THRESHOLD && proximity_readings[7] > P_THRESHOLD) {
    printf("Headon\n");
    left_counter = 0;
    right_counter = 0;

    // randomly choose left or right
    if (rand() % 2) {
      turn_left(MAX_SPEED);
    } else {
      turn_right(MAX_SPEED);
    }
    passive_wait(1.0); // turn for 1 second

  } else if (proximity_readings[0] > P_THRESHOLD) {
    printf("FR-Block\n");
    left_counter = 0;
    right_counter = 0;

    turn_left(MAX_SPEED/2);

  } else if (proximity_readings[7] > P_THRESHOLD) {
    printf("FL-Block\n");
    left_counter = 0;
    right_counter = 0;

    turn_right(MAX_SPEED/2);

  } else {
    printf("no obstacles\n");
    left_counter = 0;
    right_counter = 0;

    move_forward(MAX_SPEED/2);
  }

}


int main(int argc, char **argv) {
  init();

  while (wb_robot_step(TIME_STEP) != -1) {
    glow_leds();
    // wb_led_set(wb_robot_get_device("led0"), 1); // led0 is epuck led1
    // wb_led_set(wb_robot_get_device("led2"), 1); // led2 is epuck led3
    // wb_led_set(wb_robot_get_device("led4"), 1); // led4 is epuck led5
    // wb_led_set(wb_robot_get_device("led6"), 1); // led6 is epuck led7

    // set rgb leds to green
    // wb_led_set(wb_robot_get_device("led1"), 0x00ff00);
    get_sensor_values();

    if (left_counter > 2 && right_counter > 2) {
      recovery();
      left_counter = 0;
      right_counter = 0;
    }

    obs_avoidance();
    passive_wait(0.1);
  }

  wb_robot_cleanup();

  return 0;
}

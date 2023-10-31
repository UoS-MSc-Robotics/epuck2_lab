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
#include <string.h>

#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>


#define MAX_SPEED 6.27/2
#define THRESHOLD 90.0
#define TIME_STEP 64


const char *distance_sensor_names[8] = {
  "ps0", "ps1", "ps2", "ps3",
  "ps4", "ps5", "ps6", "ps7"
};

WbDeviceTag leftMotor;
WbDeviceTag rightMotor;
WbDeviceTag distance_sensor[8];

double sensorValues[8] = {0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0};
double sensorWeight[8] = {0.0, 0.0, 0.0, 0.0,
                          0.0, 0.0, 0.0, 0.0};

double proximityWeights[3] = {0.8, 0.6, 0.6}; // [ps0, ps1, ps2] && [ps7, ps6, ps5]

double leftSpeed = 0.0;
double rightSpeed = 0.0;


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
    sensorValues[i] = wb_distance_sensor_get_value(distance_sensor[i]);
  }
}

/*
 * Function run the initialization process
*/
void init() {
  wb_robot_init();

  leftMotor = wb_robot_get_device("left wheel motor");
  rightMotor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(leftMotor, INFINITY);
  wb_motor_set_position(rightMotor, INFINITY);
  wb_motor_set_velocity(leftMotor, 1.0);
  wb_motor_set_velocity(rightMotor, 1.0);

  for (int i = 0; i < 8; i++) {
    distance_sensor[i] = wb_robot_get_device(distance_sensor_names[i]);
    wb_distance_sensor_enable(distance_sensor[i], TIME_STEP);
  }
}



int main(int argc, char *argv[]) {
  init();

  while (wb_robot_step(TIME_STEP) != -1) {

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

    wb_motor_set_velocity(leftMotor, leftSpeed);
    wb_motor_set_velocity(rightMotor, rightSpeed);

  }
    wb_robot_cleanup();
    return 0;
}

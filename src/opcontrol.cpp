#include "main.h"

#include <cmath>
#include <algorithm>
#include <sstream>

using namespace pros;

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


bool sort(const void *arg1, const void *arg2) {
  //Dereference off a C-like cast to double pointer
  double first = *(double*)arg1;
  double second = *(double*)arg2;

  return (first < second);
}

void XDrive(void *p) {
  Controller cont(E_CONTROLLER_MASTER);


  //Code is written assuming +Power on all motors turns robot clockwise
  //Motor 1: Front left, Motor 2: Front right - Motors named in clockwise direction
  Motor drive1(DrivePort1);
  Motor drive2(DrivePort2);
  Motor drive3(DrivePort3);
  Motor drive4(DrivePort4);

  double[4] powerList = {0, 0, 0, 0};
  double power1, power2, power3, power4;

  while (true) {
    int leftY = cont.get_analog(ANALOG_LEFT_Y);
    int leftX = cont.get_analog(ANALOG_LEFT_X);
    int rightX = cont.get_analog(ANALOG_RIGHT_X);

    if (leftX < noStrafes) leftX = 0;

    //Pre-scale calculations:
    powerList[0] = leftY + leftX + rightX;
    powerList[1] = -leftY + leftX + rightX;
    powerList[2] = -leftY - leftX + rightX;
    powerList[3] = leftY -leftX + rightX;

    double maxVal = std::max(powerList, *sort);

    for (int i=0; i<4; i++) {
      powerList[i] /= (maxVal / 127.0); //Ensure double type
    }

    drive1.move(powerList[0]); drive2.move(powerList[1]);
    drive3.move(powerList[2]); drive4.move(powerList[3]);

    pros::delay(20);
  }
}


void opcontrol() {

}

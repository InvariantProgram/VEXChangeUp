#include "main.h"

#include <cmath>
#include <algorithm>
#include <array>


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

bool absComp{

}


ADIEncoder rightEnc(RightEncTop, RightEncBot, false);
ADIEncoder leftEnc(LeftEncTop, LeftEncBot, false);
ADIEncoder horEnc(HorEncTop, HorEncBot, false);

void setState(State state) {

}
void resetSensors() {
    rightEnc.reset();
    leftEnc.reset();
    horEnc.reset();
}

void XDrive(void *p) {
  Controller cont(E_CONTROLLER_MASTER);


  //Code is written assuming +Power on all motors turns robot clockwise
  //Motor 1: Front left, Motor 2: Front right - Motors named in clockwise direction
  Motor drive1(FrontLeftWheelPort, true);
  Motor drive2(FrontRightWheelPort, true);
  Motor drive3(BackRightWheelPort, true);
  Motor drive4(BackLeftWheelPort, true);

  std::array <double, 4> powerList = {0, 0, 0, 0};
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

    double maxVal = *(std::max_element(powerList.begin(), powerList.end()));

    for (int i=0; i<4; i++) {
      powerList[i] /= (maxVal / 127.0); //Ensure double type
    }

    drive1.move(powerList[0]); drive2.move(powerList[1]);
    drive3.move(powerList[2]); drive4.move(powerList[3]);

    pros::delay(20);
  }
}


void opcontrol() {
    Chassis newChassis{ 2.75, 13, 0.5 };
    Sensor_vals valStorage{ 0, 0, 0, true };

    ThreeTrackerOdom odomSys(newChassis);

    OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);
    display.setStateCallback(setState);
    display.setResetCallback(resetSensors);

    std::string driveTaskName("Drive Task");
    Task driveTask(XDrive, &driveTaskName);

    while (true) {
        int LVal = leftEnc.get_value(); int RVal = rightEnc.get_value(); int HVal = horEnc.get_value();
        int LDiff = LVal - valStorage.left;
        int RDiff = RVal - valStorage.right;
        int HDiff = HVal - valStorage.middle;
        std::array<int, 3> tickDiffs{ LDiff, RDiff, HDiff };
        valStorage.setVals(RVal, LVal, HVal);

        odomSys.odomStep(tickDiffs);
        display.setData(odomSys.getState(), valStorage);

        pros::delay(20);
    }
}

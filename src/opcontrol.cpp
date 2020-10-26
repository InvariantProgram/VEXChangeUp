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

void XDrive(void *p) {
  Controller cont(E_CONTROLLER_MASTER);


  //Code is written assuming +Power on all motors turns robot clockwise
  //Motor 1: Front left, Motor 2: Front right - Motors named in clockwise direction
  Motor FrontLeftWheelMotor(FrontLeftWheelPort, E_MOTOR_GEARSET_36, 1);
  Motor FrontRightWheelMotor(FrontRightWheelPort, E_MOTOR_GEARSET_36, 1);
  Motor BackRightWheelMotor(BackRightWheelPort, E_MOTOR_GEARSET_36, 1);
  Motor BackLeftWheelMotor(BackLeftWheelPort, E_MOTOR_GEARSET_36, 1);

  Motor LeftIntakeMotor(LeftIntakePort, E_MOTOR_GEARSET_18, 0);
  Motor RightIntakeMotor(RightIntakePort, E_MOTOR_GEARSET_18, 1);
  Motor UptakeMotor(UptakePort, E_MOTOR_GEARSET_06, 0);
  UptakeMotor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

  Motor IndexerMotor(IndexerPort, E_MOTOR_GEARSET_06, 1);
  IndexerMotor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

  std::array <double, 4> powerList = {0, 0, 0, 0};
  // double power1, power2, power3, power4;

  while (true) {
    int leftY = cont.get_analog(ANALOG_LEFT_Y);
    int leftX = cont.get_analog(ANALOG_LEFT_X);
    int rightX = cont.get_analog(ANALOG_RIGHT_X);

    int RunIntake = IntakePower * (cont.get_digital(E_CONTROLLER_DIGITAL_R1) - cont.get_digital(E_CONTROLLER_DIGITAL_R2));
    // R1/+ is intake, R2/- is outtake, runs at IntakePower
    int RunIndexer = IndexerPower * cont.get_digital(E_CONTROLLER_DIGITAL_L1);
    // L1 is indexer
    int RunUptake = ((! RunIndexer) ? RunIntake : RunIndexer);
    // Runs with indexer, if indexer not running then runs with intake



    if ((leftX < noStrafes) && (leftX > -noStrafes))
      leftX = 0;

    //Pre-scale calculations:
    powerList[0] = leftY + leftX + rightX;
    powerList[1] = -leftY + leftX + rightX;
    powerList[2] = -leftY - leftX + rightX;
    powerList[3] = leftY - leftX + rightX;

    double maxVal = *(std::max_element(powerList.begin(), powerList.end()));

    for (int i = 0; i < 4; i++)
      powerList[i] /= (abs((int) maxVal) / 127.0); //Ensure double type

    FrontLeftWheelMotor.move(powerList[0]); FrontRightWheelMotor.move(powerList[1]);
    BackRightWheelMotor.move(powerList[2]); BackLeftWheelMotor.move(powerList[3]);

    LeftIntakeMotor.move(RunIntake);
    RightIntakeMotor.move(RunIntake);
    UptakeMotor.move(RunUptake);
    IndexerMotor.move(RunIndexer);

    pros::delay(20);
  }
}


void opcontrol() {
    std::string driveTaskName("Drive Task");
    Task driveTask(XDrive, &driveTaskName);
}

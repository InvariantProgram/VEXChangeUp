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

bool absComp(const double &a, const double &b)
{
  bool FirstLess = false;

  if (abs((int) a) < abs((int) b))
    FirstLess = true;

  return(FirstLess);
}

int ScaleRawJoystick(int raw)
{
  // formula swiped from https://www.vexforum.com/t/what-do-you-think-is-a-more-efficient-way-to-drive-your-robot/64857/42
  int ScaledVal = (pow(raw, JoystickScaleConst)) / (pow(127, JoystickScaleConst - 1));
  if ((JoystickScaleConst % 2 == 0) && (raw < 0))
    ScaledVal *= -1;

  return(ScaledVal);
}

void XDrive(void *p) {
  Controller cont(E_CONTROLLER_MASTER);

  //Code is written assuming +Power on all motors turns robot clockwise
  //Motor 1: Front left, Motor 2: Front right - Motors named in clockwise direction
  Motor FrontLeftWheelMotor(FrontLeftWheelPort, E_MOTOR_GEARSET_18, 0);
  Motor FrontRightWheelMotor(FrontRightWheelPort, E_MOTOR_GEARSET_18, 0);
  Motor BackRightWheelMotor(BackRightWheelPort, E_MOTOR_GEARSET_18, 0);
  Motor BackLeftWheelMotor(BackLeftWheelPort, E_MOTOR_GEARSET_18, 0);

/*
  calculate goal voltages from joystick values
  slew
*/

  std::array <int, 4> Voltage = {0};
  std::array <int, 4> GoalVoltage = {0};

  while (true)
  {
    int leftY = cont.get_analog(ANALOG_LEFT_Y);
    int leftX = cont.get_analog(ANALOG_LEFT_X);
    int rightY = cont.get_analog(ANALOG_RIGHT_Y);

    bool strafing = (abs(leftX) > StrafeDeadzone);

    if (abs(leftY - rightY) < forceStraight) rightY = leftY;
    leftY = ScaleRawJoystick(leftY);
    rightY = ScaleRawJoystick(rightY);
    if (leftX > 0)
      leftX = ScaleRawJoystick(127 * (leftX - StrafeDeadzone) / (127 - StrafeDeadzone));
    else if (leftX < 0)
      leftX = ScaleRawJoystick(127 * (leftX + StrafeDeadzone) / (127 - StrafeDeadzone));

    GoalVoltage[0] = leftY + (leftX * strafing);
    GoalVoltage[1] = -rightY + (leftX * strafing);
    GoalVoltage[2] = -rightY - (leftX * strafing);
    GoalVoltage[3] = leftY - (leftX * strafing);

    int MaxVoltage = *(std::max_element(GoalVoltage.begin(), GoalVoltage.end(), absComp));

    if (abs(MaxVoltage) > 127) // scales if necessary for strafing
    {
      for (int i = 0; i < 4; i++)
        GoalVoltage[i] /= (abs((int) MaxVoltage) / 127.0); // truncates but who cares
    }

    for (int i = 0; i < 4; ++i)
      Voltage[i] += MovementScale * (GoalVoltage[i] - Voltage[i]); // busted?
    

    FrontLeftWheelMotor.move(Voltage[0]);
    FrontRightWheelMotor.move(Voltage[1]);
    BackRightWheelMotor.move(Voltage[2]);
    BackLeftWheelMotor.move(Voltage[3]);

    pros::delay(20);
  }
}

void intake(void* p) {
    Controller cont(E_CONTROLLER_MASTER);

    Distance maxIntake(botDist);
    Distance topUptake(topDist);

    Motor leftIntake(LeftIntakePort, E_MOTOR_GEARSET_18, 0);
    Motor rightIntake(RightIntakePort, E_MOTOR_GEARSET_18, 1);
    Motor rightUptake(rightUptakePort, E_MOTOR_GEARSET_06, 1);
    Motor leftUptake(leftUptakePort, E_MOTOR_GEARSET_06, 0);

    bool blocked = false;


    while (true) {
        

        bool R2 = cont.get_digital(E_CONTROLLER_DIGITAL_R2);
        bool R1 = cont.get_digital(E_CONTROLLER_DIGITAL_R1);
        bool L2 = cont.get_digital(E_CONTROLLER_DIGITAL_L2);
        bool L1 = cont.get_digital(E_CONTROLLER_DIGITAL_L1);

        bool Y = cont.get_digital(E_CONTROLLER_DIGITAL_Y);
        bool Right = cont.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT);
        bool Left = cont.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT);

        double intakeVal = IntakePower * L2 - reversePower * (L1 || R1);
        rightIntake.move(intakeVal);
        leftIntake.move(intakeVal);

        if (Y) {
            rightIntake.move_velocity(200);
            leftIntake.move_velocity(200);
            rightUptake.move_velocity(300);
            leftUptake.move_velocity(300);
            pros::delay(250);
            rightUptake.move_velocity(0);
            leftUptake.move_velocity(0);
            rightIntake.move_velocity(0);
            leftIntake.move_velocity(0);
        }
        if (Right) {
            rightUptake.move_velocity(600);
            leftUptake.move_velocity(600);
            pros::delay(750);
            rightIntake.move_velocity(200);
            leftIntake.move_velocity(200);
            pros::delay(150);
            rightIntake.move_velocity(0);
            leftIntake.move_velocity(0);
            pros::delay(100);
            rightUptake.move_velocity(0);
            leftUptake.move_velocity(0);
        }
        

        if (R2) {
            rightUptake.move_velocity(UptakePower);
            leftUptake.move_velocity(UptakePower);
        }
        else if (R1) {
            rightUptake.move(-reversePower);
            leftUptake.move(-reversePower);
        }
        else {
            rightUptake.move_velocity(0);
            leftUptake.move_velocity(0);
        }

        pros::delay(1);
    }


}

void showOdomDriver(void* p) {
    OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);

    pros::ADIEncoder rightEnc(RightEncTop, RightEncBot);
    pros::ADIEncoder leftEnc(LeftEncTop, LeftEncBot, true);
    pros::ADIEncoder horEnc(HorEncTop, HorEncBot, true);

    Sensor_vals valStorage{ 0, 0, 0, true };
    Chassis newChassis{ 2.75, 13, 0.5 };
    ThreeTrackerOdom odomSys(newChassis);

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

void opcontrol() {
    std::string driveTaskName("Drive Task");
    std::string intakeTaskName("Intake Task");
    Task driveTask(XDrive, &driveTaskName);
    Task intakeTask(intake, &intakeTaskName);
    Task odomShow(showOdomDriver);
}

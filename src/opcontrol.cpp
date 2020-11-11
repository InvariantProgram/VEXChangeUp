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
    raw *= -1;

  return(ScaledVal);
}

void XDrive(void *p) {
  Controller cont(E_CONTROLLER_MASTER);

  //Code is written assuming +Power on all motors turns robot clockwise
  //Motor 1: Front left, Motor 2: Front right - Motors named in clockwise direction
  Motor FrontLeftWheelMotor(FrontLeftWheelPort, E_MOTOR_GEARSET_18, 1);
  Motor FrontRightWheelMotor(FrontRightWheelPort, E_MOTOR_GEARSET_18, 1);
  Motor BackRightWheelMotor(BackRightWheelPort, E_MOTOR_GEARSET_18, 1);
  Motor BackLeftWheelMotor(BackLeftWheelPort, E_MOTOR_GEARSET_18, 1);

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
      Voltage[i] += MovementScale * (GoalVoltage[i] - Voltage[i]);

    FrontLeftWheelMotor.move(Voltage[0]);
    FrontRightWheelMotor.move(Voltage[1]);
    BackRightWheelMotor.move(Voltage[2]);
    BackLeftWheelMotor.move(Voltage[3]);

    pros::delay(20);
  }

/*
  std::array <double, 4> powerList = {0, 0, 0, 0};
  std::array <double, 4> velList = {0, 0, 0, 0};

  while (true) {
    int leftY = cont.get_analog(ANALOG_LEFT_Y);
    int leftX = cont.get_analog(ANALOG_LEFT_X);
    int rightY = cont.get_analog(ANALOG_RIGHT_Y);

    if (abs(leftX) < StrafeDeadzone)
      leftX = 0;

    velList[0] = FrontLeftWheelMotor.get_actual_velocity();
    velList[1] = FrontRightWheelMotor.get_actual_velocity();
    velList[2] = BackRightWheelMotor.get_actual_velocity();
    velList[3] = BackLeftWheelMotor.get_actual_velocity();

    printf("BackRightWheelMotor get_voltage returns: %f\n", BackRightWheelMotor.get_actual_velocity());
    printf("rightY: %d\n", rightY);

    for (int i = 0; i < 4; i++) {
        double diff = powerList[i] - velList[i];
        if (powerList[i] == 0)
            continue;
        else {
            if (abs(diff) > 5) powerList[i] = velList[i] + diff * MovementScale;
        }
    }

    //Pre-scale calculations:
    powerList[0] = leftY + leftX;
    powerList[1] = -rightY + leftX;
    powerList[2] = -rightY - leftX;
    powerList[3] = leftY - leftX;

    double maxVal = *(std::max_element(powerList.begin(), powerList.end(), absComp));

    for (int i = 0; i < 4; i++)
      powerList[i] /= (abs((int) maxVal) / 127.0); //Ensure double type

    FrontLeftWheelMotor.move(powerList[0]); FrontRightWheelMotor.move(powerList[1]);
    BackRightWheelMotor.move(powerList[2]); BackLeftWheelMotor.move(powerList[3]);
    pros::delay(20);
  }
  */
}

void intake(void* p) {
    Controller cont(E_CONTROLLER_MASTER);

    Motor LeftIntakeMotor(LeftIntakePort, E_MOTOR_GEARSET_06, 0);
    Motor RightIntakeMotor(RightIntakePort, E_MOTOR_GEARSET_06, 1);
    Motor UptakeMotor(UptakePort, E_MOTOR_GEARSET_06, 0);
    UptakeMotor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    Motor IndexerMotor(IndexerPort, E_MOTOR_GEARSET_06, 0);
    IndexerMotor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    while (true) {

        int RunIntake = IntakePower * (cont.get_digital(E_CONTROLLER_DIGITAL_L2) - cont.get_digital(E_CONTROLLER_DIGITAL_L1));
        // L1/+ is intake, L2/- is outtake, runs at IntakePower
        int RunIndexer = IndexerPower * cont.get_digital(E_CONTROLLER_DIGITAL_R2);
        // R2 is indexer
        int RunUptake = 0;
        if (cont.get_digital(E_CONTROLLER_DIGITAL_R1))
            RunUptake = 1;
        else if (RunIndexer || (RunIntake > 0))
            RunUptake = -1;
        RunUptake *= UptakePower;


        LeftIntakeMotor.move(RunIntake);
        RightIntakeMotor.move(RunIntake);
        UptakeMotor.move(RunUptake);
        IndexerMotor.move(RunIndexer);

        pros::delay(20);
    }
}


void opcontrol() {
    std::string driveTaskName("Drive Task");
    std::string intakeTaskName("Intake Task");
    Task driveTask(XDrive, &driveTaskName);
    Task intakeTask(intake, &intakeTaskName);
}

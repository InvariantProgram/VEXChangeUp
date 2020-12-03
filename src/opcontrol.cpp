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
      Voltage[i] += MovementScale * (GoalVoltage[i] - Voltage[i]); // busted?


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

inline int RestartIndexerTimer()
{
  return(2 * 1000 / 20); // 2 seconds, measured in 20 ms intervals
}

void intake(void* p) {
    Controller cont(E_CONTROLLER_MASTER);

    Motor LeftIntakeMotor(LeftIntakePort, E_MOTOR_GEARSET_18, 0);
    Motor RightIntakeMotor(RightIntakePort, E_MOTOR_GEARSET_18, 1);
    Motor UptakeMotor(UptakePort, E_MOTOR_GEARSET_06, 0);
      UptakeMotor.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    Motor IndexerMotor(IndexerPort, E_MOTOR_GEARSET_06, 0);
      IndexerMotor.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    ADIAnalogIn ScoreLineSensor(ScoreLineSensorPort);
    ScoreLineSensor.calibrate();
    ADIAnalogIn TopSlotLineSensor(TopSlotLineSensorPort);
    TopSlotLineSensor.calibrate();

    int IndexerTimer = 0;
    int BallsToScore = 0;
    int prevBallsToScore = 0;
    bool ScoreSensorFoundBall = 0;
    bool R2WasPressed = 0;

    while (true) {
        bool L1 = cont.get_digital(E_CONTROLLER_DIGITAL_L1);
        bool L2 = cont.get_digital(E_CONTROLLER_DIGITAL_L2);
        bool R1 = cont.get_digital(E_CONTROLLER_DIGITAL_R1);
        bool R2 = cont.get_digital(E_CONTROLLER_DIGITAL_R2);
        printf("%d %d %d %d\n", L1, L2, R1, R2);

        int ScoreSensorVal = ScoreLineSensor.get_value();
        int TopSlotSensorVal = TopSlotLineSensor.get_value();

        if (R2 && !R2WasPressed) // counts R2 presses
        {
          ++BallsToScore;
          IndexerTimer = RestartIndexerTimer();
        }

        // If a ball has just been ejected:
        if (ScoreSensorVal < SCORE_LINE_SENSOR_LIMIT && !ScoreSensorFoundBall) // ball has just been ejected
        {
          --BallsToScore;
          // Are there still balls left?
          if (BallsToScore > 0)
            // Yes.  Restart timer.
            IndexerTimer = RestartIndexerTimer();
          else
            // No.  Stop timer.
            IndexerTimer = 0;
        }

        // If timer hits 0:
        if (IndexerTimer == 0)
          // Stop running indexer.
          BallsToScore = 0;

        printf("%d\n", ScoreLineSensor.get_value_calibrated());
        printf("%d\n", BallsToScore);

        int RunIntake = (L2 - L1);      // L2 is intake, L1 is outtake

        int RunIndexer;
        if (BallsToScore > 0)
          RunIndexer = 1;
        else if (L2 && TopSlotSensorVal > TOP_SLOT_LINE_SENSOR_LIMIT)
          RunIndexer = 1;
        else
          RunIndexer = 0;

        int RunUptake = 0;
        if (R1)
            RunUptake = 1;              // R1 is filter
        else if (RunIndexer || (RunIntake > 0))
            RunUptake = -1;             // otherwise just run uptake



        LeftIntakeMotor.move(RunIntake * IntakePower);
        RightIntakeMotor.move(RunIntake * IntakePower);
        UptakeMotor.move(RunUptake * UptakePower);
        IndexerMotor.move(RunIndexer * IndexerPower);
/*
        bool L2Pressed = cont.get_digital(E_CONTROLLER_DIGITAL_L2);
        bool L1Pressed = cont.get_digital(E_CONTROLLER_DIGITAL_L2);
        int RunIndexer = 0;

        if (L2Pressed)
          timer = 50;
        if (timer > 0)
          RunIndexer = 1;
        timer -= 1;
*/
        R2WasPressed = R2 ? true : false;
        ScoreSensorFoundBall = (ScoreSensorVal < SCORE_LINE_SENSOR_LIMIT) ? true : false;
        --IndexerTimer;
        pros::delay(20);
    }
}


void opcontrol() {
    std::string driveTaskName("Drive Task");
    std::string intakeTaskName("Intake Task");
    Task driveTask(XDrive, &driveTaskName);
    Task intakeTask(intake, &intakeTaskName);
}

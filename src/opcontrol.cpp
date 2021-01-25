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
pros::ADIEncoder rightEnc(RightEncTop, RightEncBot, true);
pros::ADIEncoder leftEnc(LeftEncTop, LeftEncBot, true);
pros::ADIEncoder horEnc(HorEncTop, HorEncBot, true);

Chassis newChassis{ 2.75, 13, 0.5 };
Sensor_vals valStorage{ 0, 0, 0, true };

ThreeTrackerOdom odomSys(newChassis);

//------------------
PIDConsts forward{ 8, 0, 0.1, 0 };
PIDConsts turn{ 5, 0, 0, 0 };
PIDConsts straight{ 0, 0, 0, 0 };

PIDController driveCont(forward);
PIDController turnCont(turn);
PIDController straightCont(straight);

XDrive testDrive(&odomSys, &driveCont, &turnCont, &straightCont, &rightEnc, &leftEnc, &horEnc,
    { FrontRightWheelPort, BackRightWheelPort }, { -FrontLeftWheelPort, -BackLeftWheelPort }, 1, 30);

void setState(State state) {
    odomSys.setState(state);
}
void resetSensors() {
    rightEnc.reset();
    leftEnc.reset();
    horEnc.reset();
}

bool absComp(const double& a, const double& b)
{
    return (abs((int)a) < abs((int)b));
}

int ScaleRawJoystick(int raw)
{
    // formula swiped from https://www.vexforum.com/t/what-do-you-think-is-a-more-efficient-way-to-drive-your-robot/64857/42
    int ScaledVal = (pow(raw, JoystickScaleConst)) / (pow(127, JoystickScaleConst - 1));
    if ((JoystickScaleConst % 2 == 0) && (raw < 0))
        raw *= -1;

    return(ScaledVal);
}

void baseControl(void* p) {
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

    std::array <double, 4> Voltage = { 0 };
    std::array <int, 4> GoalVoltage = { 0 };

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

        GoalVoltage[0] = leftY * (1 - strafing) + (leftX * strafing);
        GoalVoltage[1] = -rightY * (1 - strafing) + (leftX * strafing);
        GoalVoltage[2] = -rightY * (1 - strafing) - (leftX * strafing);
        GoalVoltage[3] = leftY * (1 - strafing) - (leftX * strafing);

        int MaxVoltage = *(std::max_element(GoalVoltage.begin(), GoalVoltage.end(), absComp));

        if (abs(MaxVoltage) > 127) // scales if necessary for strafing
        {
            for (int i = 0; i < 4; i++)
                GoalVoltage[i] /= (abs((int)MaxVoltage) / 127.0); // truncates but who cares
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

void runOdom(void* p) {
    OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);
    display.setStateCallback(setState);
    display.setResetCallback(resetSensors);

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
    Task odomTask(runOdom);

    testDrive.driveDistance(15);

    
}

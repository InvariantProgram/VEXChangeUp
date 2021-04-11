#include "main.h"
#include <array>

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

pros::ADIEncoder rightEnc(RightEncTop, RightEncBot);
pros::ADIEncoder leftEnc(LeftEncTop, LeftEncBot);
pros::ADIEncoder horEnc(HorEncTop, HorEncBot, true);

Chassis newChassis{ 2.75, 12.75, 0.5 };
Sensor_vals valStorage{ 0, 0, 0, true };

ThreeTrackerOdom odomSys(newChassis);

PIDConsts straight{ 17, 0, 0, 0 };
PIDConsts turn{ 160, 0, 0, 0 };
PIDController driveCont(straight);
PIDController turnCont(turn);

XDrive newX({ FrontRightWheelPort, BackRightWheelPort }, { -FrontLeftWheelPort, -BackLeftWheelPort });

PursuitController chassisController(&newX, &odomSys, &driveCont, &turnCont);

PathFollower Rohith(&chassisController);


double convertToRadians(double input) {
    return input / 180 * 3.14159;
}

void robotTask(void* p) {
    newX.changeGearset(pros::E_MOTOR_GEARSET_06);
    
    Point p1 = { 0, 0 }, p2 = { 10, 0 }, p3 = { 20, 20 }, p4 = { 25, 20 };
    Spline newspline({ p1, p2, p3, p4 });

    Rohith.insert(newspline, 30, 50);
    
    Rohith.logStates();

    Rohith.execute();
}

void odomTask(void* p) {
    std::array<int, 3> tickDiffs;
    OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);

    //odomSys.setState({0, 0, convertToRadians(90)});

    while (true) {
        int LVal = leftEnc.get_value(); int RVal = rightEnc.get_value(); int HVal = horEnc.get_value();
        int LDiff = LVal - valStorage.left;
        int RDiff = RVal - valStorage.right;
        int HDiff = HVal - valStorage.middle;
        tickDiffs = { LDiff, RDiff, HDiff };
        valStorage.setVals(RVal, LVal, HVal);

        odomSys.odomStep(tickDiffs);
        display.setData(odomSys.getState(), valStorage);



        pros::delay(20);
    }
}

void opcontrol() {
    pros::Task dispOdom(odomTask);
    pros::Task runRobot(robotTask);
}

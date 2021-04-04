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

pros::ADIEncoder rightEnc(RightEncTop, RightEncBot, true);
pros::ADIEncoder leftEnc(LeftEncTop, LeftEncBot, true);
pros::ADIEncoder horEnc(HorEncTop, HorEncBot, true);

Chassis newChassis{ 2.75, 12.75, 0.5 };
Sensor_vals valStorage{ 0, 0, 0, true };

ThreeTrackerOdom odomSys(newChassis);

PIDConsts straight{ 3, 0, 0, 0 };
PIDConsts turn{ 80, 0, 0, 0 };
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


    Rohith.changeDecrementProp(3);

    double initTime = pros::millis();
    Point p1{ 0, 0 }, p2{ 0, 25 }, p3{ 15, 20 }, p4{ 5,25 };
    Point p5{ 5, 25 }, p6{ -5, 35 }, p7{ 50, 50 }, p8{ 50, 70 };
    std::array<Point,4> iPoints =  { p1, p2, p3, p4 };
    Spline testSpline(iPoints);
    iPoints = {p5, p6, p7, p8};
    Spline spline2(iPoints);
    Rohith.insert(testSpline, 50);
    Rohith.insert(spline2, 50);
    double timeTaken = pros::millis() - initTime;

    printf("Time Taken: %f ms\n", timeTaken);

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

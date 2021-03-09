#include "main.h"

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
pros::ADIEncoder leftEnc(LeftEncTop, LeftEncBot, true);
pros::ADIEncoder horEnc(HorEncTop, HorEncBot);

Chassis newChassis{ 2.75, 12.75, 0.5 };
Sensor_vals valStorage{ 0, 0, 0, true };

ThreeTrackerOdom odomSys(newChassis);

PIDConsts straight{ 9.25, 0, 0, 0 };
PIDConsts turn{ 125, 0, 0, 0 };
PIDController driveCont(straight);
PIDController turnCont(turn);

XDrive newX({ -FrontRightWheelPort, -BackRightWheelPort }, { FrontLeftWheelPort, BackLeftWheelPort });

PursuitController chassisController(&newX, &odomSys, &driveCont, &turnCont);

pros::Motor endMotor(RightIntakePort);


double convertToRadians(double input) {
    return input / 180 * 3.14159;
}

void robotTask(void* p) {

    State targetState{ 8, 13 , convertToRadians(315) };
    chassisController.toPoint(targetState);
    pros::delay(100);
    newX.forwardVelocity(500, 200);
    pros::delay(1000);
    newX.forwardVelocity(650, -200);
    pros::delay(100);
    chassisController.toPoint({-33 , 15 , convertToRadians(270)});
    pros::delay(100);
    newX.forwardVelocity(400, 200);
    pros::delay(1000);
    chassisController.toPoint({-45 , 35 , convertToRadians(230)});
    pros::delay(100);
    chassisController.toPoint({-73.5 , 2.8 , convertToRadians(237)});
    pros::delay(100);newX.forwardVelocity(500, 200);
    pros::delay(1000);






    //pros::delay(10000);
    /*
    State targetState{ 24, 0, 0 };
    chassisController.toPoint(targetState);

    //State targetState{ 25, -10, convertToRadians(225) };
    //chassisController.toPoint(targetState);
    pros::delay(100);
    //chassisController.toPoint({ 25, -10, convertToRadians(225) });
    pros::delay(100000);
    newX.forwardVelocity(500, 200);
    pros::delay(750);
    newX.forwardVelocity(650, -200);
    pros::delay(100);
    //chassisController.toPoint({ 36, -6, convertToRadians(90) });
    //pros::delay(100);
    chassisController.toPoint({ 29, 33, convertToRadians(180) });
    newX.forwardVelocity(350, 200);
    pros::delay(750);
    chassisController.toPoint({ 45, 33, convertToRadians(125) });
    pros::delay(100);
    chassisController.toPoint({ 33, 68, convertToRadians(125) });
    pros::delay(100);
    chassisController.toPoint({ 12, 84, convertToRadians(155) });

    newX.forwardVelocity(500, 200);
    pros::delay(750);
    */
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

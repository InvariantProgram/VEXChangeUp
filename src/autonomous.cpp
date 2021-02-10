#include "main.h"

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

pros::ADIEncoder rightEnc(RightEncTop, RightEncBot);
pros::ADIEncoder leftEnc(LeftEncTop, LeftEncBot);
pros::ADIEncoder horEnc(HorEncTop, HorEncBot, true);

Chassis newChassis{ 2.75, 13, 0.5 };
Sensor_vals valStorage{ 0, 0, 0, true };

ThreeTrackerOdom odomSys(newChassis);

//------------------
PIDConsts straight{ 8, 0, 0.1, 0 };
PIDConsts turn{ 8, 0, 0, 0 };
PIDConsts nonExist{ 0, 0, 0, 0 };

PIDConsts longForward{ 5, 0, 0.001, 0 };

PIDController driveCont(straight);
PIDController turnCont(turn);
PIDController straightCont(nonExist);

XDrive drive(&odomSys, &driveCont, &turnCont, &straightCont, &rightEnc, &leftEnc, &horEnc,
    { -FrontRightWheelPort, -BackRightWheelPort }, { FrontLeftWheelPort, BackLeftWheelPort }, 1, 10);

pros::Motor LeftIntakeMotor(LeftIntakePort, pros::E_MOTOR_GEARSET_18, 0);
pros::Motor RightIntakeMotor(RightIntakePort, pros::E_MOTOR_GEARSET_18, 1);

pros::Motor leftIntake(LeftIntakePort, pros::E_MOTOR_GEARSET_18, 0);
pros::Motor rightIntake(RightIntakePort, pros::E_MOTOR_GEARSET_18, 1);
pros::Motor rightUptake(rightUptakePort, pros::E_MOTOR_GEARSET_06, 1);
pros::Motor leftUptake(leftUptakePort, pros::E_MOTOR_GEARSET_06, 0);

void runIntake(int power) {
    rightIntake.move_velocity(power);
    leftIntake.move_velocity(power);
}
void runUptake(int power) {
    rightUptake.move_velocity(power);
    leftUptake.move_velocity(power);
}
void flipIntake() {
    rightIntake.move_velocity(200);
    leftIntake.move_velocity(200);
    pros::delay(350);
    rightIntake.move_velocity(0);
    leftIntake.move_velocity(0);
}

void systemTask(void* p) {
    flipIntake();
}
void driveTask(void* p) {
    drive.driveDistance(17);
    runIntake(50);
    drive.turnAngle(-90);
    drive.driveDistance(10);
    runIntake(0);
    runUptake(600);
    drive.runallMotors(300, 75);
    pros::delay(750);
    runUptake(0);
    drive.runallMotors(300, -75);
    runIntake(200);
    pros::delay(150);
    driveCont.setGains(longForward);
    drive.driveDistance(-10);
    drive.turnPoint({ -10, 10 });
    drive.driveDistance(30);
    runIntake(0);
    pros::delay(300);
    drive.runallMotors(500, 75);
    runUptake(600);
    pros::delay(750);
    runUptake(0);
    
}
void odomTask(void* p) {
    OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);
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


void autonomous() {
    pros::Task system(systemTask);
    pros::Task drive(driveTask);
    pros::Task odomShow(odomTask);
}
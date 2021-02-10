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

pros::ADIEncoder rightEnc(RightEncTop, RightEncBot, true);
pros::ADIEncoder leftEnc(LeftEncTop, LeftEncBot, true);
pros::ADIEncoder horEnc(HorEncTop, HorEncBot, true);

Chassis newChassis{ 2.75, 13, 0.5 };
Sensor_vals valStorage{ 0, 0, 0, true };

ThreeTrackerOdom odomSys(newChassis);

//------------------
PIDConsts straight{ 8, 0, 0.1, 0 };
PIDConsts turn{ 5, 0, 0, 0 };

PIDController driveCont(straight);
PIDController turnCont(turn);

XDrive drive(&odomSys, &driveCont, &turnCont, &rightEnc, &leftEnc, &horEnc,
    { FrontRightWheelPort, BackRightWheelPort }, { -FrontLeftWheelPort, -BackLeftWheelPort }, 1, 10);

pros::Motor LeftIntakeMotor(LeftIntakePort, pros::E_MOTOR_GEARSET_18, 0);
pros::Motor RightIntakeMotor(RightIntakePort, pros::E_MOTOR_GEARSET_18, 1);
pros::Motor UptakeMotor(UptakePort, pros::E_MOTOR_GEARSET_06, 0);
pros::Motor IndexerMotor(IndexerPort, pros::E_MOTOR_GEARSET_06, 0);

void runIntake(int power) {
    LeftIntakeMotor.move(power);
    RightIntakeMotor.move(power);
}
void flipOut() {
    UptakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    IndexerMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    runIntake(-127);
    UptakeMotor.move(127);
    pros::delay(750);
    runIntake(0);
    UptakeMotor.move(0);
}
void score() {
    double lastSensorVal;
    IndexerMotor.move(127);
    while (ScoreLineSensor.get_value() > SCORE_LINE_SENSOR_LIMIT && lastSensorVal < SCORE_LINE_SENSOR_LIMIT) {
        lastSensorVal = ScoreLineSensor.get_value();
        pros::delay(20);
    }
    pros::delay(500);
    IndexerMotor.move_velocity(0);
}
void toPosition() {
    while (TopSlotLineSensor.get_value() > TOP_SLOT_LINE_SENSOR_LIMIT) {
        IndexerMotor.move(100);
        pros::delay(20);
    }
    IndexerMotor.move(0);
}
void systemTask(void* p) {
    pros::delay(300);
    flipOut();
    pros::delay(500);
    runIntake(127);
    UptakeMotor.move(-127);
    pros::delay(2000);
    toPosition();
    pros::delay(1750);
    score();
    score();
    runIntake(0);
    score();
    score();
    pros::delay(1750);
    UptakeMotor.move_velocity(-127);
    runIntake(127);
    pros::delay(5000);
    toPosition();
    pros::delay(2500);
    score();
    pros::delay(750);
    runIntake(0);
    score();
    pros::delay(500);
    runIntake(127);
}
void driveTask(void* p) {
    drive.strafeDistance(7.8);
    turn = { 25, 0, 0, 0 };
    drive.turnAngle(0);
    pros::delay(750);
    drive.driveDistance(26);
    turn = { 15, 0, 0, 0 };
    drive.turnAngle(45);
    pros::delay(20);
    drive.driveDistance(10);
    drive.runallMotors(900, 100);
    pros::delay(1500);
    drive.driveDistance(-10);
    drive.turnAngle(215);
    drive.driveDistance(54);
    drive.turnAngle(90);
    drive.driveDistance(10);
    drive.runallMotors(900, 100);
    pros::delay(1500);
    drive.driveDistance(-6);
    drive.turnAngle(180);
    drive.driveDistance(32);
    drive.turnAngle(135);
    drive.driveDistance(10);
    drive.runallMotors(900, 100);
    pros::delay(1500);
    drive.driveDistance(-10);
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
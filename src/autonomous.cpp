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

extern std::string selectedAuton;

pros::Mutex notification;

pros::ADIEncoder rightEnc(RightEncTop, RightEncBot);
pros::ADIEncoder leftEnc(LeftEncTop, LeftEncBot, true);
pros::ADIEncoder horEnc(HorEncTop, HorEncBot, true);

Chassis newChassis{ 2.75, 12.75, 0.5 };
Sensor_vals valStorage{ 0, 0, 0, true };

ThreeTrackerOdom odomSys(newChassis);

//------------------
PIDConsts straight{ 8, 0, 0.0001, 0 };
PIDConsts turn{ 9.25, 0, 0, 0 };
PIDConsts nonExist{ 0, 0, 0, 0 };

PIDConsts longForward{ 8.75, 0, 0.001, 0 };

PIDController driveCont(straight);
PIDController turnCont(turn);
PIDController straightCont(nonExist);

XDrive drive(&odomSys, &driveCont, &turnCont, &straightCont, &rightEnc, &leftEnc, &horEnc,
    { -FrontRightWheelPort, -BackRightWheelPort }, { FrontLeftWheelPort, BackLeftWheelPort }, 1, 30);

pros::Motor LeftIntakeMotor(LeftIntakePort, pros::E_MOTOR_GEARSET_18, 0);
pros::Motor RightIntakeMotor(RightIntakePort, pros::E_MOTOR_GEARSET_18, 1);

pros::Motor leftIntake(LeftIntakePort, pros::E_MOTOR_GEARSET_06, 0);
pros::Motor rightIntake(RightIntakePort, pros::E_MOTOR_GEARSET_06, 1);
pros::Motor rightUptake(rightUptakePort, pros::E_MOTOR_GEARSET_06, 1);
pros::Motor leftUptake(leftUptakePort, pros::E_MOTOR_GEARSET_06, 0);

pros::Distance botDistance(botDist);
pros::Distance topDistance(topDist);

void runIntake(int power) {
    rightIntake.move_velocity(power);
    leftIntake.move_velocity(power);
}
void runUptake(int power) {
    rightUptake.move_velocity(power);
    leftUptake.move_velocity(power);
}
void flipIntake() {
    runIntake(200);
    pros::delay(300);
    runIntake(-200);
    pros::delay(150);
    rightIntake.move_velocity(0);
    leftIntake.move_velocity(0);
}
void intakeToMax(int speed, int timeOut = 1000) {
    double startTime = pros::millis();
    runIntake(speed);
    int i = 0; bool run = true;
    while (pros::millis() - startTime < timeOut && run) {
        if (botDistance.get() < detectLimit) run = false;
        pros::delay(5);
    }
    runIntake(0);
}
void index(int timeOut = 2500) {
    double startTime = pros::millis();
    runUptake(100);
    int i = 0; bool run = true;
    while (pros::millis() - startTime < timeOut && run) {
        if (topDistance.get() < detectLimit) run = false;
        pros::delay(5);
    }
    runUptake(0);
}
void fullIntake() {
    double startTime = pros::millis();
    runUptake(150);
    runIntake(600);
    int i = 0; bool run = true;
    while (pros::millis() - startTime < 2500 && run) {
        if (topDistance.get() < detectLimit) run = false;
        pros::delay(5);
    }
    runUptake(0);
    runIntake(0);
}

void systemTask(void* p) {
    if (selectedAuton == "None") {
        flipIntake();
        pros::delay(500);
        notification.take(TIMEOUT_MAX);
        runIntake(600);
        pros::delay(750);
        runIntake(0);
        pros::delay(250);
        index();
        notification.give();
        notification.take(TIMEOUT_MAX);
        intakeToMax(600, 1500);
        notification.give();
    }
}
void driveTask(void* p) {
    if (selectedAuton == "None") {
        notification.take(0);
        drive.driveDistance(17);
        drive.turnAngle(-90);
        drive.driveDistance(14);
        drive.runallMotors(300, 60);
        runUptake(600);
        pros::delay(700);
        runUptake(0);
        pros::delay(150);
        notification.give();
        drive.driveDistance(-10);
        drive.turnAngle(135);
        driveCont.setGains(longForward);
        drive.driveDistance(47);
        drive.turnAngle(-135);
        drive.runallMotors(500, 60);
        runUptake(600);
        pros::delay(400);
        runUptake(0);
        notification.take(0);
        driveCont.setGains(straight);
        drive.driveDistance(-10);
        drive.turnAngle(135);
        driveCont.setGains(longForward);
        drive.driveDistance(40);
        drive.turnAngle(180);
        runIntake(200);
        driveCont.setGains(straight);
        drive.driveDistance(24);
        runIntake(0);
        drive.runallMotors(500, 60);
        notification.give();
        notification.take(2500);
        runIntake(300);
        runUptake(600);
        pros::delay(900);
        runUptake(0);
        pros::delay(500);
        runIntake(0);
        drive.runallMotors(600, -75);
    }
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
    pros::Task newSystems(systemTask);
    pros::Task baseControl(driveTask);
    pros::Task odomShow(odomTask);
}
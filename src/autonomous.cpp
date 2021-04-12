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

extern std::string selectedAuton;

pros::Mutex notification;

pros::ADIEncoder rightEnc(RightEncTop, RightEncBot);
pros::ADIEncoder leftEnc(LeftEncTop, LeftEncBot);
pros::ADIEncoder horEnc(HorEncTop, HorEncBot);

Chassis newChassis{ 2.75, 12.75, 0.5 };
Sensor_vals valStorage{ 0, 0, 0, true };

ThreeTrackerOdom odomSys(newChassis);

PIDConsts straight{ 12, 0, 0, 0 };
PIDConsts turn{ 200, 0, 0, 0 };


PIDController driveCont(straight);
PIDController turnCont(turn);

XDrive newX({ FrontRightWheelPort, BackRightWheelPort }, { -FrontLeftWheelPort, -BackLeftWheelPort });

PursuitController chassisController(&newX, &odomSys, &driveCont, &turnCont);

PathFollower fullChassis(&chassisController);

pros::Motor leftIntake(LeftIntakePort, pros::E_MOTOR_GEARSET_06, 0);
pros::Motor rightIntake(RightIntakePort, pros::E_MOTOR_GEARSET_06, 1);
pros::Motor rightUptake(rightUptakePort, pros::E_MOTOR_GEARSET_06, 1);
pros::Motor leftUptake(leftUptakePort, pros::E_MOTOR_GEARSET_06, 0);

pros::Distance botDistance(botDist);
pros::Distance topDistance(topDist);

//Helper Functions:
//To stop intake/uptake runIntake(0) or runUptake(0)
void runIntake(int power) {
    rightIntake.move_velocity(power);
    leftIntake.move_velocity(power);
}
void runUptake(int power) {
    rightUptake.move_velocity(power);
    leftUptake.move_velocity(power);
}
void flipOut() {
    runIntake(-600);
    pros::delay(250);
    runIntake(0);    
}
//Adjust detectLimit as necessary
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
void index(int timeOut = 1500) {
    double startTime = pros::millis();
    runUptake(200);
    bool run = true;
    while (pros::millis() - startTime < timeOut && run) {
        pros::delay(5);
        if (topDistance.get() < detectLimit) run = false;
    }
    runUptake(0);
}
void outtake(int time = 750) {
    runIntake(-300);
    runUptake(-300);
    pros::delay(time);
    runIntake(0);
    runUptake(0);
}
void outtakeToBall(int balls, int timeout=1250) {
    ballsIn = 0;
    runIntake(-200);
    runUptake(-300);
    int startTime = pros::millis();
    while (ballsIn < balls && pros::millis() - startTime < timeout) {
        pros::delay(20);
    }
    runIntake(0);
    runUptake(0);
}

double convertToRadians(double input) {
    return input / 180 * 3.14159;
}

int ballsIn = 0;
int ballsOut = 0;

void ballCounter(void* p) {
    double lastVal = detectLimit + 50;
    double curVal = 0;

    double lastTop = topDetect + 50;
    double curTop = 0;
    while (true) {
        curVal = botDistance.get();
        if (curVal <= 0) curVal = lastVal;
        if (curVal < detectLimit && lastVal > detectLimit)
            ballsIn++;
        lastVal = curVal;

        curTop = topDistance.get();
        if (curTop <= 0) curTop = lastTop;
        if (curTop < topDetect && lastTop > topDetect)
            ballsOut++;
        lastTop = curTop;

        pros::delay(20);
    }
}

void Scoreballs(int shootBalls, int inBalls, int delay=2500) {
    ballsIn = 0; ballsOut = 0;
    runIntake(400);
    runUptake(200);
    double initTime = pros::millis();
    bool intGo = true, outGo = true;
    while ((pros::millis() - initTime) < delay && (intGo || outGo)) {
        if (ballsIn >= inBalls) {
            runIntake(0);
            intGo = false;
        }
        if (ballsOut > shootBalls) {
            runUptake(0);
            outGo = false;
        }
    }
    runIntake(0);
    runUptake(0);
}
void subsystemSynchronous(void* p) {
}

void robotTask(void* p) {
    rightUptake.set_brake_mode(MOTOR_BRAKE_HOLD);
    leftUptake.set_brake_mode(MOTOR_BRAKE_HOLD);
    rightIntake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    leftIntake.set_brake_mode(MOTOR_BRAKE_BRAKE);

    newX.changeGearset(pros::E_MOTOR_GEARSET_06);
    
    
    flipOut();
    pros::delay(50);

    runIntake(600);
    Point p1 = { 0,0 }, p2 = { 17, 0 }, p3 = { 33, -18 }, p4 = { 42, -18 };
    Spline spline1({ p1, p2, p3, p4 });
    fullChassis.insert(spline1, 30, 1250);
    fullChassis.execute();
    
    pros::delay(20);

    fullChassis.insert({ 34, -13.5, convertToRadians(45) }, 750);
    fullChassis.execute();

    runIntake(0);

    newX.forwardVelocity(750, 200);
    Scoreballs(2, 2);
    pros::delay(100);
    newX.forwardVelocity(500, -250);  

    outtake(1000);

    runIntake(600);
    fullChassis.insert({ 10, -38, convertToRadians(300) }, 500);
    fullChassis.insert({2.5, -36, convertToRadians(300)}, 700);
    p1 = { -1, -36.5 }; p2 = { 0, -38.5 }; p3 = { 18, -55 }; p4 = { 31, -55 };
    Spline spline2({ p1, p2, p3, p4 });
    fullChassis.insert(spline2, 30, 1000);
    fullChassis.execute();

    runIntake(0);
    newX.forwardVelocity(700, 200);
    Scoreballs(2, 1);
    pros::delay(100);
    newX.forwardVelocity(300, -200);
}

void odomTask(void* p) {
    std::array<int, 3> tickDiffs;
    OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);

    lv_obj_t* label = lv_label_create(lv_scr_act(), NULL);
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

        std::string text = "ballsIn: " + std::to_string(ballsIn) + " ballsOut: " + std::to_string(ballsOut);
        lv_label_set_text(label, text.c_str());

        pros::delay(20);
    }
}

void autonomous() {
    pros::Task incBalls(ballCounter);
    pros::Task subSystems(subsystemSynchronous); //commented back in when we learn how to use lmao
    pros::Task dispOdom(odomTask);
    pros::Task runRobot(robotTask);
}

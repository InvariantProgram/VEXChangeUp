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

int phase = 0;

pros::ADIEncoder rightEnc(RightEncTop, RightEncBot);
pros::ADIEncoder leftEnc(LeftEncTop, LeftEncBot);
pros::ADIEncoder horEnc(HorEncTop, HorEncBot);

Chassis newChassis{ 2.75, 13.1675, 6.25 };
Sensor_vals valStorage{ 0, 0, 0, true };

ThreeTrackerOdom odomSys(newChassis);

PIDConsts turnless{ 150, 0, 0, 0 };

PIDConsts straight{ 11, 0, 0, 0 };
PIDConsts turn{ 185, 0, 0, 0 };

PIDConsts skillsStraight{ 22, 0, 0.0002, 0 };
PIDConsts skillsTurn{ 200, 0, .2, 0 };

PIDConsts skillsStraight2{ 17.5, 0, 0.002, 0 };
PIDConsts skillsTurn2{ 200, 0, .2, 0 };

PIDConsts skillsStraight3{ 16, 0, 0.001, 0 };
PIDConsts skillsTurn3{ 270, 0, .2, 0 };

PIDConsts skillsStraight4{ 19.5, 0, 0.0002, 0 };
PIDConsts skillsTurn4{ 250, 0, .2, 0 };

PIDConsts skillsStraight5{ 19.5, 0, 0.0002, 0 };
PIDConsts skillsTurn5{ 235, 0, .2, 0 };

PIDConsts skillsStraight6{ 8, 0, 0.0002, 0 };
PIDConsts skillsTurn6{ 235, 0, .2, 0 };

PIDConsts skillsStraight7{ 16.5, 0, 0.0002, 0 };
PIDConsts skillsTurn7{ 250, 0, .2, 0 };

PIDConsts skillsStraight8{ 16.5, 0, 0.0002, 0 };
PIDConsts skillsTurn8{ 275, 0, .2, 0 };


PIDConsts auto1{ 18, 0, 0.00001, 0 };
PIDConsts auto2{ 19.75, 0, 0, 0 };

PIDConsts turn2{ 220, 0, 0, 0 };

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
pros::Distance shotDistance(shotDist);

std::array<double, 4> forwardVel = { 75, 75, 75, 75 };
std::array<double, 4> slowerForward = { 100, 100, 100, 100 };
std::array<double, 4> fasterForwardVel = { 150, 150, 150, 150 };
std::array<double, 4> maxSpeed = { 600, 600, 600, 600 };


int ballsIn = 0;
int ballsOut = 0;

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
    pros::delay(500);
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
    runUptake(100);
    bool run = true;
    while (pros::millis() - startTime < timeOut && run) {
        pros::delay(5);
        if (topDistance.get() < detectLimit) run = false;
    }
    runUptake(0);
}
void outtake(int time = 750) {
    runIntake(-400);
    runUptake(-400);
    pros::delay(time);
    runIntake(0);
    runUptake(0);
}

void outtakeGoal2(int time = 750) {
    runIntake(-600);
    runUptake(-600);
    pros::delay(time);
    runIntake(0);
    runUptake(0);
}


void outtop(int time = 750) {
    runIntake(-600);
    runUptake(600);
    pros::delay(time);
    runIntake(0);
    runUptake(0);
}

double convertToRadians(double input) {
    return input / 180 * 3.14159;
}

void ballCounter(void* p) {
    double lastVal = detectLimit + 50;
    double curVal = 0;

    double lastTop = shotDetect + 50;
    double curTop = 0;
    while (true) {
        curVal = botDistance.get();
        if (curVal <= 0) curVal = lastVal;
        if (curVal < detectLimit && lastVal > detectLimit)
            ballsIn++;
        lastVal = curVal;

        curTop = shotDistance.get();
        if (curTop <= 0) curTop = lastTop;
        if (curTop < shotDetect && lastTop > shotDetect)
            ballsOut++;
        lastTop = curTop;

        pros::delay(10);
    }
}

void scoreIntakes(int inBalls, int delay = 2500) {
    ballsIn = 0;
    runIntake(300);
    runUptake(600);
    double initTime = pros::millis();
    bool intGo = true;
    while (intGo && (pros::millis() - initTime) < delay) {
        if (ballsIn >= inBalls) {
            runIntake(0);
            intGo = false;
        }
        pros::delay(10);
    }
    runIntake(0);
    pros::delay(200);
    runUptake(0);
}
void shoot(int shootBalls, int delay = 1000) {
    ballsOut = 0;
    runUptake(375);
    double initTime = pros::millis();
    bool go = true;
    while (go && (pros::millis() - initTime) < delay) {
        if (ballsOut >= shootBalls) go = false;
        pros::delay(10);
    }
    runUptake(0);
}


void score3Balls(int delay = 2500) {
    ballsIn = 0; ballsOut = 0;
    runIntake(300);
    runUptake(600);
    double initTime = pros::millis();
    bool intGo = true, upGo = true;
    while ((intGo || upGo) && (pros::millis() - initTime) < delay) {
        if (ballsIn >= 2) {
            runIntake(0);
            intGo = false;
        }
        if (ballsOut >= 4) {
            runUptake(0);
            upGo = false;
        }
    }
    runIntake(0);
    runUptake(0);
}

void scoreBalls(int shootBalls, int inBalls, int delay = 2500) {
    ballsIn = 0; ballsOut = 0;
    runIntake(450);
    runUptake(200);
    double initTime = pros::millis();
    bool intGo = true, upGo = true;
    while ((intGo || upGo) && (pros::millis() - initTime) < delay) {
        if (ballsIn >= inBalls) {
            runIntake(0);
            intGo = false;
        }
        if (ballsOut >= shootBalls) {
            runUptake(0);
            upGo = false;
        }
    }
    runIntake(0);
    pros::delay(150);
    runUptake(0);
}

void midSkills() {
    ballsIn = 0; ballsOut = 0;
    runIntake(600);
    runUptake(175);
    double initTime = pros::millis();
    bool intGo = true, upGo = true;
    while ((intGo || upGo) && (pros::millis() - initTime) < 1750) {
        if (ballsIn >= 3) {
            runIntake(0);
            intGo = false;
        }
        if (ballsOut >= 1) {
            runUptake(75);
            upGo = false;
        }
    }
    runIntake(0);
    runUptake(0);
}

void outtakeToBall(int balls, int timeout = 1250) {
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

void scoreAuto(int inBalls, int beforeShot = 700, int timeOut = 2000) {
    ballsIn = 0;
    runIntake(300);
    runUptake(30);
    pros::delay(beforeShot);
    runUptake(600);
    bool intGo = true;
    double initTime = pros::millis();
    while (intGo && (pros::millis() - initTime) < timeOut) {
        if (ballsIn >= inBalls) intGo = false;
    }
    runIntake(0);
    pros::delay(200);
    runUptake(0);
}

void delayUntilPhase(int waitUntil) {
    while (phase < waitUntil) {
        pros::delay(20);
    }
}



void subsystemSynchronous(void* p) {
    rightUptake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    leftUptake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rightIntake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    leftIntake.set_brake_mode(MOTOR_BRAKE_BRAKE);


    flipOut();

    if (selectedAuton == "Right Side (9, 6)") {
        delayUntilPhase(1);
        pros::delay(125);
        scoreAuto(3, 1100);
        phase = 2;
        pros::delay(350);
        outtake(1000);
    }
    else if (selectedAuton == "Left to Mid (7, MID)") {
        delayUntilPhase(1);
        pros::delay(125);
        scoreAuto(3, 800, 1500);
        phase = 2;
        pros::delay(275);
        outtake(1500);
        pros::delay(100);
        runIntake(600);

        delayUntilPhase(3);
        runIntake(600);
        runUptake(50);
        pros::delay(250);
        runIntake(0);
        runUptake(0);

        delayUntilPhase(4);
        index(2000);
    }
    else if (selectedAuton == "Home Row") {
        delayUntilPhase(1);
        pros::delay(150);
        scoreAuto(3, 1100);
        phase = 2;
        pros::delay(600);
        outtake(1000);

        delayUntilPhase(3);
        pros::delay(350);
        outtakeGoal2(1250);
        pros::delay(100);
        runIntake(600);

        delayUntilPhase(4);
        scoreAuto(3, 700, 1000);
        phase = 5;
        pros::delay(350);
        runIntake(600);
        runUptake(35);
        pros::delay(450);
        runIntake(0);
        runUptake(0);
    }
    else if (selectedAuton == "Home Row Jiggle") {
        delayUntilPhase(1);
        scoreBalls(1, 1);
        phase = 2;
        pros::delay(300);
        runIntake(500);
        runUptake(30);
        pros::delay(600);
        runIntake(0);
        runUptake(0);

        delayUntilPhase(3);
        scoreBalls(1, 0);
        phase = 4;

        delayUntilPhase(5);
        pros::delay(350);
        outtakeGoal2(1250);
        pros::delay(100);
        runIntake(600);

        delayUntilPhase(6);
        intakeToMax(400, 1000);
        scoreBalls(1, 0);
        phase = 7;
        pros::delay(350);
        runIntake(600);
        runUptake(35);
        pros::delay(450);
        runIntake(0);
        runUptake(0);
    }
    else if (selectedAuton == "9 to 8") {
        delayUntilPhase(1);
        pros::delay(150);
        scoreAuto(3, 1300);
        phase = 2;
        pros::delay(600);
        outtake(1000);

        delayUntilPhase(3);
        scoreAuto(2, 750, 2500);
        phase = 4;

        delayUntilPhase(5);
        pros::delay(350);
        outtakeGoal2(1250);
        pros::delay(100);
        runIntake(600);
    }
    else {
        delayUntilPhase(1);
        pros::delay(150);
        scoreAuto(3, 1100);
        phase = 2;
        pros::delay(600);
        outtake(1000);

        delayUntilPhase(3);
        pros::delay(350);
        outtakeGoal2(1250);
        pros::delay(100);
        runIntake(600);

        delayUntilPhase(4);
        scoreAuto(3);
        phase = 5;
        pros::delay(350);
        runIntake(600);
        runUptake(35);
        pros::delay(450);
        runIntake(0);
        runUptake(0);
    }
}

/*
* 7: Score3Balls OR scoreBalls(3, 2) - meh?
* 4: scoreIntakes(1)
* 1: scoreBalls(1, 2)
* 2: scoreBalls(1, 1)
*/


void robotTask(void* p) {

    driveCont.setGains({ 18.5, 0, 0.00001, 0 });
    turnCont.setGains({ 180, 0, 0, 0 });

    newX.changeGearset(pros::E_MOTOR_GEARSET_06);

    double startTime = pros::millis();

    //Full Cycle 9 and 6
    if (selectedAuton == "Right Side (9,6)") {
        driveCont.setGains({ 18.5, 0, 0.00001, 0 });
        turnCont.setGains({ 180, 0, 0, 0 });

        fullChassis.insert({ 12.5, 14.5, convertToRadians(311) }, 500);
        fullChassis.insert({ 15, 8.5, convertToRadians(313) }, 200); //15,6
        chassisController.changeFloorVel(50);
        fullChassis.execute();


        phase = 1;
        newX.forwardVelocity(750, 200);
        newX.forwardVelocity(250, -100);
        newX.forwardVelocity(250, 100);
        newX.forwardVelocity(250, -100);
        newX.forwardVelocity(250, 100);
        delayUntilPhase(2);
        newX.forwardVelocity(300, -200);

        driveCont.setGains({ 16, 0, 0.00001, 0 });
        turnCont.setGains({ 207, 0, .2, 0 });
        fullChassis.insert({ 9, 16.5, convertToRadians(313) }, 200);
        fullChassis.execute();

        driveCont.setGains({ 16, 0, 0.00001, 0 });
        turnCont.setGains({ 207, 0, .2, 0 });
        fullChassis.insert({ 9, 16.5, convertToRadians(50) }, 500);
        Point p1 = { 9, 16.5 }, p2 = { 11, 28 }, p3 = { 18.5, 37 }, p4 = { 24.75, 45 }; //59
        Spline spline1({ p1, p2, p3, p4 });
        fullChassis.insert(spline1, 30, 700);
        fullChassis.execute();

        pros::delay(100);
        newX.forwardVelocity(325, 100);

        scoreBalls(2, 4, 5000);

        newX.forwardVelocity(325, -100);
    }

     //7 then 2 ball clamp mid
    else if (selectedAuton == "Left to Mid (7, MID)") {
        driveCont.setGains({ 18.5, 0, 0.00001, 0 });
        turnCont.setGains({ 180, 0, 0, 0 });

        fullChassis.insert({ 12.5, -14.5, convertToRadians(49) }, 500);
        fullChassis.insert({ 14.5, -8.5, convertToRadians(45) }, 200); //15,6
        chassisController.changeFloorVel(50);
        fullChassis.execute();

        phase = 1;
        newX.forwardVelocity(750, 200);
        pros::delay(350);
        newX.forwardVelocity(250, -100);
        newX.forwardVelocity(250, 100);
        newX.forwardVelocity(250, -100);
        newX.forwardVelocity(250, 100);
        delayUntilPhase(2);
        newX.forwardVelocity(300, -200);

        pros::delay(200);

        driveCont.setGains({ 15, 0, 0.00001, 0 });
        turnCont.setGains({ 207, 0, .2, 0 });
        fullChassis.insert({ 0, -10, convertToRadians(300) }, 200);
        Point p1 = { 0, -12 }, p2 = { 3,-15 }, p3 = { 8.65, -18.5 }, p4 = { 8.65, -45 };
        Spline spline1({ p1, p2, p3, p4 });
        fullChassis.insert(spline1, 35, 700);
        fullChassis.execute();

        phase = 3;
        pros::delay(250);
        driveCont.setGains({ 12, 0, 0.00001, 0 });
        fullChassis.insert({ -7, -47.25, convertToRadians(275) }, 500);
        fullChassis.execute();

        newX.strafeVelocity(350, 150);

        phase = 4;
        fullChassis.insert({ -14.5, -31.5, convertToRadians(240) }, 500);
        fullChassis.insert({ -26.75, -38, convertToRadians(278) }, 650);
        fullChassis.execute();

        runIntake(600);
        fullChassis.insert({ -27, -40.5, convertToRadians(280) }, 150);
        fullChassis.execute();


        runIntake(300);
        newX.forwardVelocity(650, 150);

        runUptake(100);
        pros::delay(800);

        runIntake(-50);
        shoot(1);
        pros::delay(150);
        shoot(1);

        
        while (pros::millis() - startTime < 14600) {
            pros::delay(20);
        }
        runIntake(-200);
        newX.forwardVelocity(300, -350);
        runIntake(0);
    }


    //HOME ROW w/ MID

    else if (selectedAuton == "Home Row") {
        driveCont.setGains({ 18.5, 0, 0.00001, 0 });
        turnCont.setGains({ 180, 0, 0, 0 });

        fullChassis.insert({ 12.5, 14.5, convertToRadians(311) }, 500);
        fullChassis.insert({ 16.25, 8.5, convertToRadians(313) }, 200); //15,6
        chassisController.changeFloorVel(50);
        fullChassis.execute();

        phase = 1;
        newX.forwardVelocity(750, 200);
        delayUntilPhase(2);
        newX.forwardVelocity(300, -200);

        driveCont.setGains(auto2);
        turnCont.setGains(turn2);
        fullChassis.insert({ -28.75, 10.5, convertToRadians(270) }, 500);
        chassisController.changeFloorVel(100);
        fullChassis.execute();

        newX.forwardVelocity(500, 200);
        scoreBalls(1, 2, 1500);

        phase = 3;
        newX.forwardVelocity(500, -200);
        pros::delay(350);

        runIntake(600);
        fullChassis.insert({ -38, 25, convertToRadians(265) }, 400);
        fullChassis.insert({ -60, 25, convertToRadians(240) }, 400);
        fullChassis.insert({ -76.5, 6.5, convertToRadians(225) }, 400); //-76.5
        fullChassis.execute();


        phase = 4;
        newX.forwardVelocity(600, 200);
        delayUntilPhase(5);
        newX.forwardVelocity(500, -200);

        if (pros::millis() - startTime > 13300) {
            chassisController.toAngle(convertToRadians(90));
            pros::delay(3000);
        }

        driveCont.setGains({ 16, 0, 0, 0 });
        turnCont.setGains({ 225, 0, 0, 0 });
        fullChassis.insert({ -52, 45.5, convertToRadians(93.5) }, 750); //-52, 45.5, 93.5
        chassisController.changeFloorVel(50);
        fullChassis.execute();

        newX.strafeVelocity(350, 200);
    }
    //Jiggle at goals; no mid
    else if (selectedAuton == "Home Row Jiggle") {
        driveCont.setGains({ 18.5, 0, 0.00001, 0 });
        turnCont.setGains({ 180, 0, 0, 0 });

        fullChassis.insert({ 12.5, 14.5, convertToRadians(311) }, 500);
        fullChassis.insert({ 16.25, 8.5, convertToRadians(313) }, 200); //15,6
        chassisController.changeFloorVel(50);
        fullChassis.execute();

        phase = 1;
        newX.forwardVelocity(750, 200);
        delayUntilPhase(2);
        newX.forwardVelocity(300, -200);

        driveCont.setGains(auto2);
        turnCont.setGains(turn2);
        fullChassis.insert({ -29, 10.5, convertToRadians(270) }, 500);
        chassisController.changeFloorVel(100);
        fullChassis.execute();

        phase = 3;
        newX.forwardVelocity(600, 200);
        delayUntilPhase(4);
        newX.forwardVelocity(300, -200);
        phase = 5;

        fullChassis.insert({ -38, 25, convertToRadians(265) }, 400);
        fullChassis.insert({ -60, 25, convertToRadians(240) }, 400);
        fullChassis.insert({ -76.5, 6.5, convertToRadians(225) }, 400); //-76.5
        fullChassis.execute();

        phase = 6;
        newX.forwardVelocity(700, 200);
        delayUntilPhase(7);
        newX.forwardVelocity(600, -200);
    }
    else if (selectedAuton == "9 to 8") {
        driveCont.setGains({ 18.5, 0, 0.00001, 0 });
        turnCont.setGains({ 180, 0, 0, 0 });

        fullChassis.insert({ 12.5, 14.5, convertToRadians(311) }, 500);
        fullChassis.insert({ 16.25, 8.5, convertToRadians(313) }, 200); //15,6
        chassisController.changeFloorVel(50);
        fullChassis.execute();

        phase = 1;
        newX.forwardVelocity(750, 200);
        newX.forwardVelocity(250, 100);
        newX.forwardVelocity(250, -100);
        newX.forwardVelocity(350, 100);
        delayUntilPhase(2);
        newX.forwardVelocity(300, -200);

        driveCont.setGains(auto2);
        turnCont.setGains(turn2);
        fullChassis.insert({ -29, 10.5, convertToRadians(270) }, 500);
        chassisController.changeFloorVel(100);
        fullChassis.execute();

        phase = 3;
        newX.forwardVelocity(600, 200);
        pros::delay(300);
        newX.forwardVelocity(250, -100);
        newX.forwardVelocity(250, 100);
        newX.forwardVelocity(250, -100);
        newX.forwardVelocity(250, 100);
        delayUntilPhase(4);
        newX.forwardVelocity(300, -200);
        phase = 5;

        fullChassis.insert({ -5, 20, convertToRadians(90) }, 100);
        fullChassis.execute();
        runIntake(0);
    }
    else {
        driveCont.setGains({ 18.5, 0, 0.00001, 0 });
        turnCont.setGains({ 180, 0, 0, 0 });

        fullChassis.insert({ 12.5, 14.5, convertToRadians(311) }, 500);
        fullChassis.insert({ 16.25, 8.5, convertToRadians(313) }, 200); //15,6
        chassisController.changeFloorVel(50);
        fullChassis.execute();


        phase = 1;
        newX.forwardVelocity(750, 200);
        delayUntilPhase(2);
        newX.forwardVelocity(300, -200);

        driveCont.setGains(auto2);
        turnCont.setGains(turn2);
        fullChassis.insert({ -29, 7, convertToRadians(270) }, 500);
        chassisController.changeFloorVel(100);
        fullChassis.execute();

        newX.forwardVelocity(500, 200);
        scoreBalls(1, 2, 1500);

        phase = 3;
        newX.forwardVelocity(500, -200);
        pros::delay(350);

        runIntake(600);
        fullChassis.insert({ -38, 25, convertToRadians(265) }, 400);
        fullChassis.insert({ -65, 25, convertToRadians(240) }, 400);
        fullChassis.insert({ -76.5, 6.5, convertToRadians(225) }, 400);
        fullChassis.execute();


        phase = 4;
        newX.forwardVelocity(600, 200);
        delayUntilPhase(5);
        newX.forwardVelocity(300, -200);

        driveCont.setGains({ 16, 0, 0, 0 });
        turnCont.setGains({ 225, 0, 0, 0 });
        fullChassis.insert({ -52, 45.5, convertToRadians(93.5) }, 750);
        chassisController.changeFloorVel(50);
        fullChassis.execute();

        newX.strafeVelocity(350, 200);
    }
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

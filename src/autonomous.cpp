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

int phase=0;

pros::ADIEncoder rightEnc(RightEncTop, RightEncBot);
pros::ADIEncoder leftEnc(LeftEncTop, LeftEncBot);
pros::ADIEncoder horEnc(HorEncTop, HorEncBot);

Chassis newChassis{ 2.75, 12.75, 3.5 };
Sensor_vals valStorage{ 0, 0, 0, true };

ThreeTrackerOdom odomSys(newChassis);

PIDConsts straight{ 12, 0, 0, 0 };
PIDConsts turn{ 175, 0, 0, 0 };

PIDConsts skillsStraight{ 7, 0, 0.01, 0 };
PIDConsts skillsTurn{ 250, 0, .2, 0 };

PIDConsts skillsStraight2{ 11, 0, 0.05, 0 };
PIDConsts skillsTurn2{ 250, 0, .2, 0 };

PIDConsts skillsTurn3{ 260, 0, .2, 0 };

PIDConsts auto1{18, 0, 0.00001, 0};
PIDConsts auto2{19.75, 0, 0, 0};

PIDConsts turn2{220, 0, 0, 0};

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
    runUptake(175);
    bool run = true;
    while (pros::millis() - startTime < timeOut && run) {
        pros::delay(5);
        if (topDistance.get() < detectLimit) run = false;
    }
    runUptake(0);
}
void outtake(int time = 750) {
    runIntake(-600);
    runUptake(-600);
    pros::delay(time);
    runIntake(0);
    runUptake(0);
}
void outtop(int time=750) {
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
    double curVal = 1;

    double lastTop = topDetect + 50;
    double curTop = 1;
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

        pros::delay(10);
    }
}

void scoreBalls(int shootBalls, int inBalls, int delay=2500) {
    ballsIn = 0; ballsOut = 0;
    runIntake(400);
    runUptake(250);
    double initTime = pros::millis();
    bool intGo = true, upGo = true;
    while ((intGo || upGo) && (pros::millis() - initTime) < delay) {
        if (ballsIn >= inBalls) {
            runIntake(0);
            intGo = false;
        }
        if (ballsOut > shootBalls) {
            runUptake(0);
            upGo = false;
        }
    }
    runIntake(0);
    runUptake(0);
}


void score3Balls(int shootBalls, int inBalls, int delay=2500) {
    ballsIn = 0; ballsOut = 0;
    runIntake(300);
    runUptake(200);
    double initTime = pros::millis();
    bool intGo = true, upGo = true;
    while ((intGo || upGo) && (pros::millis() - initTime) < delay) {
        if (ballsIn >= inBalls) {
            runIntake(0);
            intGo = false;
        }
        if (ballsOut > shootBalls) {
            runUptake(0);
            upGo = false;
        }
    }
    runIntake(0);
    runUptake(0);
}

void scoreBallsSkills(int shootBalls, int inBalls, int delay=2500) {
    ballsIn = 0; ballsOut = 0;
    runIntake(300);
    runUptake(175);
    double initTime = pros::millis();
    bool intGo = true, upGo = true;
    while ((intGo || upGo) && (pros::millis() - initTime) < delay) {
        if (ballsIn >= inBalls) {
            runIntake(0);
            intGo = false;
        }
        if (ballsOut > shootBalls) {
            runUptake(0);
            upGo = false;
        }
    }
    runIntake(0);
    runUptake(0);
}


void scoreAuto(int inBalls, int timeOut = 2000) {
  ballsIn = 0; ballsOut = 0;
  runIntake(400);
  runUptake(600);
  bool intGo = true;
  double initTime = pros::millis();
  while (intGo && (pros::millis() - initTime) < timeOut) {
      if (ballsIn >= inBalls) intGo = false;
  }
  runIntake(0);
  pros::delay(400);
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


void delayUntilPhase(int waitUntil) {
  while (phase < waitUntil) {
    pros::delay(20);
  }
}

void subsystemSynchronous(void* p) {
  rightUptake.set_brake_mode(MOTOR_BRAKE_HOLD);
  leftUptake.set_brake_mode(MOTOR_BRAKE_HOLD);
  rightIntake.set_brake_mode(MOTOR_BRAKE_BRAKE);
  leftIntake.set_brake_mode(MOTOR_BRAKE_BRAKE);

  flipOut();

  delayUntilPhase(1);
  pros::delay(200);
  outtake(1000);
  runIntake(600);

  /*
  delayUntilPhase(2);
  pros::delay(200);
  outtop(1000);
  pros::delay(100);
  runIntake(600);
  */
}

void robotTask(void* p) {
  newX.changeGearset(pros::E_MOTOR_GEARSET_06);

/*
  driveCont.setGains(auto1);
  chassisController.changeFloorVel(50);

  fullChassis.insert({18.5, -12.5, convertToRadians(50)}, 200);
  fullChassis.execute();

  newX.forwardVelocity(750, 200);
  scoreAuto(3);

  phase=1;

  fullChassis.insert({0, -10, convertToRadians(300)}, 600);
  Point p1={3, -12}, p2={5,-15}, p3={7,-27}, p4={7, -32};
  Spline spline1({p1, p2, p3, p4});
  fullChassis.insert(spline1, 30, 700);
  fullChassis.execute();
  newX.forwardVelocity(300, 125);

  fullChassis.insert({-3, -30, convertToRadians(240)}, 200);
  fullChassis.execute();
  newX.forwardVelocity(300, 125);

  newX.forwardVelocity(400, -200);

  fullChassis.insert({-14, -18, convertToRadians(270)}, 300);
  fullChassis.execute();

  newX.forwardVelocity(400, 150);
  */



  /*
  driveCont.setGains(auto1);
  chassisController.changeFloorVel(50);

  fullChassis.insert({11.5, 17, convertToRadians(310)}, 265);
  fullChassis.insert({18.5, 10.5, convertToRadians(315)}, 200);
  fullChassis.execute();

  newX.forwardVelocity(750, 200);
  scoreBalls(2, 2, 1700);
  newX.forwardVelocity(300, -200);

  phase=1;

  driveCont.setGains(auto2);
  turnCont.setGains(turn2);
  fullChassis.insert({-22, 15.5, convertToRadians(270)}, 500);
  fullChassis.execute();

  newX.forwardVelocity(500, 200);
  scoreBalls(1, 2, 1500);

  phase = 2;
  newX.forwardVelocity(500, -200);

  runIntake(600);
  driveCont.setGains(auto1);
  fullChassis.insert({-38, 45, convertToRadians(240)}, 400);
  Point p1={-33, 45}, p2={-42, 45}, p3={-64.5, 18.5}, p4={-68.8, 15.25};
  Spline spline1({p1, p2, p3, p4});
  fullChassis.insert(spline1, 30, 600);
  fullChassis.execute();

  runIntake(0);
  newX.forwardVelocity(400, 200);
  scoreBalls(2, 2);
  newX.forwardVelocity(300, -200);

  fullChassis.insert({-44, 41, convertToRadians(90)}, 900);
  fullChassis.execute();
  newX.strafeVelocity(300, -125);
  */

 //Skills

     pros::delay(800);

    runIntake(600);

    Point p1 = { 0,0 }, p2 = { 17, 0 }, p3 = { 38, -18 }, p4 = { 42, -18 };
    Spline spline1({ p1, p2, p3, p4 });
    fullChassis.insert(spline1, 30, 1250);
    chassisController.changeFloorVel(75);
    fullChassis.execute();
    runUptake(15);

    pros::delay(100);

    fullChassis.insert({ 36, -12, convertToRadians(41) }, 750);
    fullChassis.execute();

    runIntake(0);
    //GOAL 7

    newX.forwardVelocity(750, 200);
    score3Balls(3, 2,2100);
    pros::delay(100);
    newX.forwardVelocity(500, -250);

    outtake(1000);

    runIntake(600);
    fullChassis.insert({ 10, -38, convertToRadians(300) }, 500);
    fullChassis.insert({4, -36, convertToRadians(295)}, 700);
    p1 = { -1, -36.5 }; p2 = { 0, -38.5 }; p3 = { 18, -55 }; p4 = { 34.25, -54 };
    Spline spline2({ p1, p2, p3, p4 });
    fullChassis.insert(spline2, 30, 1000);
    chassisController.changeFloorVel(85);
    fullChassis.execute();
    //index();
    runUptake(15);

    //GOAL 4
    newX.forwardVelocity(500, 125);
    scoreBalls(2, 2, 1000);
    pros::delay(100);
    newX.forwardVelocity(400, -200);

    chassisController.toAngle(convertToRadians(45));
    outtake(1000);
    pros::delay(100);

    runIntake(600);
    fullChassis.insert({19, -75, convertToRadians(270)}, 700);
    fullChassis.execute();

    newX.forwardVelocity(750, 125);
    pros::delay(350);

    fullChassis.insert({44, -85, convertToRadians(0)}, 700);
    fullChassis.execute();
    newX.forwardVelocity(200, 150);

    fullChassis.insert({ 44, -87.5, convertToRadians(310) }, 500);
    fullChassis.execute();

    //GOAL 1
    newX.forwardVelocity(700, 200);
    scoreBallsSkills(1, 3, 1000);
    pros::delay(100);

    newX.forwardVelocity(500, -200);

    driveCont.setGains(skillsStraight2);
    turnCont.setGains(skillsTurn2);

    fullChassis.insert({ 5, -93.5, convertToRadians(270) }, 500);
    fullChassis.execute();

    //GOAL 2
    newX.forwardVelocity(500, 200);
    scoreBallsSkills(0, 1, 1000);
    pros::delay(100);

    phase=1;
    fullChassis.insert({ 27, -85, convertToRadians(270)}, 500);
    fullChassis.insert({ 28, -80, convertToRadians(245)}, 500);
    fullChassis.insert({ 26, -65, convertToRadians(210) }, 500);
    fullChassis.execute();
    runIntake(600);
    pros::delay(250);

    fullChassis.insert({-40 , -90, convertToRadians(220)}, 500);
    fullChassis.insert({-42 , -99, convertToRadians(223)}, 1000);
    fullChassis.insert({-43, -100, convertToRadians(220) }, 500);
    chassisController.changeFloorVel(75);
    fullChassis.execute();

    pros::delay(100);

    //GOAL 3
    newX.forwardVelocity(500, 125);
    scoreBalls(2, 2, 1000);
    pros::delay(100);
    newX.forwardVelocity(400, -200);

    chassisController.toAngle(convertToRadians(230));
    outtake(1000);
    pros::delay(100);


    runIntake(600);
    fullChassis.insert({-47, -85, convertToRadians(180) }, 1000);
    fullChassis.insert({-35, -66, convertToRadians(90) }, 500);
    fullChassis.execute();

    pros::delay(250);

    fullChassis.insert({-32, -54.5, convertToRadians(180) }, 500);
    fullChassis.execute();
    runUptake(15);

    //GOAL 6
    newX.forwardVelocity(500, 125);
    scoreBalls(2, 2, 1000);
    pros::delay(100);

    fullChassis.insert({-32, -54.5, convertToRadians(225) }, 500);
    fullChassis.execute();

    outtake(1000);
    pros::delay(250);
    runIntake(600);

    turnCont.setGains(skillsTurn3);
    fullChassis.insert({-32, -54.5, convertToRadians(0) }, 500);
    fullChassis.execute();

    pros::delay(250);
    runUptake(25);
    fullChassis.insert({-27, -54.5, convertToRadians(0) }, 500);
    fullChassis.execute();


    //MID GOAL
    runIntake(300);
    newX.forwardVelocity(750, 150);

    scoreBallsSkills(1, 3);

    runIntake(-300);
    newX.forwardVelocity(650, -200);

    chassisController.toAngle(convertToRadians(120));

    outtake(1750);
    pros::delay(250);
    runIntake(600);

    fullChassis.insert({-31, -64, convertToRadians(45) }, 500);
    fullChassis.insert({-25, -57, convertToRadians(45) }, 500);
    fullChassis.insert({-25, -16, convertToRadians(90) }, 500);
    fullChassis.execute();

    //GOAL 8
    newX.forwardVelocity(500, 125);
    scoreBalls(1, 1, 1000);
    pros::delay(100);
    newX.forwardVelocity(500, -125);

    chassisController.toAngle(convertToRadians(220));
    outtake(1000);
    pros::delay(100);
    runIntake(600);
















    //TURN TO ANGLE FOR STRAIGHT LINE TO 3
    /*
    fullChassis.insert({ 10, -65, convertToRadians(210) }, 500);
    fullChassis.execute();
    */



    /*
        turnCont.setGains(skillsTurn);
        runIntake(600);
        Point p1 = { 0,0 }, p2 = { 36, -7 }, p3 = { 32, 16 }, p4 = { 40, 23 };
        Spline spline1({ p1, p2, p3, p4 });
        fullChassis.insert(spline1, 30, 1250);
        chassisController.changeFloorVel(75);
        fullChassis.execute();

    pros::delay(100000);
    */



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

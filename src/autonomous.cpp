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


PIDConsts turnless{ 150, 0, 0, 0 };

PIDConsts straight{ 12, 0, 0, 0 };
PIDConsts turn{ 175, 0, 0, 0 };

PIDConsts skillsStraight{ 7, 0, 0.01, 0 };
PIDConsts skillsTurn{ 250, 0, .2, 0 };

PIDConsts skillsStraight2{ 11, 0, 0.05, 0 };
PIDConsts skillsTurn2{ 250, 0, .2, 0 };

PIDConsts skillsStraight3{ 12, 0, 0, 0 };
PIDConsts skillsTurn3{ 270, 0, .1, 0 };

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
pros::Distance shotDistance(shotDist);


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
    runIntake(-400);
    runUptake(-400);
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

    double lastTop = detectLimit + 50;
    double curTop = 1;
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

void scoreBalls(int shootBalls, int inBalls, int delay=2500) {
    ballsOut = 0; ballsIn = 0;
    runIntake(300);
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
        pros::delay(10);
    }
    runIntake(0);
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


void score3Balls(int delay=2500) {
    ballsIn = 0; ballsOut = 0;
    runIntake(300);
    runUptake(200);
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


void scoreAuto(int inBalls, int delay=700, int timeOut = 2000) {
  ballsIn = 0;
  runIntake(300);
  runUptake(600);
  pros::delay(delay);
  bool intGo = true;
  double initTime = pros::millis();
  while (intGo && (pros::millis() - initTime) < timeOut) {
      if (ballsIn >= inBalls) intGo = false;
  }
  runIntake(0);
  pros::delay(200);
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
  rightUptake.set_brake_mode(MOTOR_BRAKE_BRAKE);
  leftUptake.set_brake_mode(MOTOR_BRAKE_BRAKE);
  rightIntake.set_brake_mode(MOTOR_BRAKE_BRAKE);
  leftIntake.set_brake_mode(MOTOR_BRAKE_BRAKE);

  flipOut();

  delayUntilPhase(1);
  scoreAuto(3);
  phase = 2;
  pros::delay(600);
  outtake(1000);

  delayUntilPhase(3);
  pros::delay(200);
  outtop(1000);
  pros::delay(100);
  runIntake(600);

  delayUntilPhase(4);
  scoreAuto(3);
  phase = 5;

  /*
  delayUntilPhase(1);
  pros::delay(450);
  outtake(1000);
  runIntake(600);

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
  double startTime = pros::millis();

  driveCont.setGains(auto1);
  chassisController.changeFloorVel(50);

  fullChassis.insert({20.5, -11.5, convertToRadians(50)}, 200);
  fullChassis.execute();

  phase = 1;
  newX.forwardVelocity(750, 200);

  delayUntilPhase(2);

  fullChassis.insert({0, -10, convertToRadians(300)}, 600);
  Point p1={3, -12}, p2={5,-15}, p3={12,-24}, p4={12, -29};
  Spline spline1({p1, p2, p3, p4});
  fullChassis.insert(spline1, 30, 700);
  fullChassis.execute();
  runIntake(600);
  newX.forwardVelocity(300, 125);

  fullChassis.insert({-4, -30, convertToRadians(240)}, 200);
  fullChassis.execute();

  newX.forwardVelocity(500, -200);

  fullChassis.insert({-19, -25, convertToRadians(270)}, 300);
  fullChassis.execute();

  newX.forwardVelocity(400, 150);

  runIntake(0);
  shoot(2);
  */

  /*
  while (pros::millis() - startTime < 14600) {
      pros::delay(20);
  }
  */

  /*
  runIntake(-100);
  newX.forwardVelocity(300, -350);
  runIntake(0);

  fullChassis.insert({ -19, -5, convertToRadians(90) }, 300);
  fullChassis.execute();
  newX.forwardVelocity(600, 200);
  scoreBalls(1, 1);
  newX.forwardVelocity(300, -200);
  */


  //Home Row
  driveCont.setGains(auto1);
  chassisController.changeFloorVel(50);

  fullChassis.insert({18.5, 10.5, convertToRadians(310)}, 200);
  fullChassis.execute();

  phase = 1;
  newX.forwardVelocity(750, 200);
  delayUntilPhase(2);
  newX.forwardVelocity(300, -200);

  driveCont.setGains(auto2);
  turnCont.setGains(turn2);
  fullChassis.insert({-21, 16.5, convertToRadians(270)}, 500);
  fullChassis.execute();

  newX.forwardVelocity(500, 200);
  scoreBalls(1, 2, 1500);

  phase = 3;
  newX.forwardVelocity(500, -200);

  runIntake(600);
  driveCont.setGains(auto1);
  fullChassis.insert({-38, 45, convertToRadians(240)}, 400);
  Point p1={-33, 45}, p2={-42, 45}, p3={-56.75, 20.5}, p4={-60.8, 17.25};
  Spline spline1({p1, p2, p3, p4});
  fullChassis.insert(spline1, 30, 700);
  fullChassis.execute();

  phase = 4;
  newX.forwardVelocity(600, 200);
  delayUntilPhase(5);
  newX.forwardVelocity(300, -200);

  fullChassis.insert({ -33, 38, convertToRadians(90) }, 500);
  fullChassis.execute();

  newX.strafeVelocity(350, 200);

 //Skills
    /*
    pros::delay(800);

    fullChassis.changeError(0.35);

    runIntake(600);

    Point p1 = { 0,0 }, p2 = { 17, 0 }, p3 = { 38, -18 }, p4 = { 42, -18 };
    Spline spline1({ p1, p2, p3, p4 });
    fullChassis.insert(spline1, 30, 1250);
    chassisController.changeFloorVel(75);
    fullChassis.execute();
    runUptake(15);

    pros::delay(100);

    fullChassis.insert({ 39.25, -11.5, convertToRadians(44) }, 750);
    fullChassis.insert({ 44.65, -7.5, convertToRadians(45) }, 200);
    fullChassis.execute();

    runIntake(0);
    //GOAL 7
    score3Balls(2000);
    pros::delay(100);
    newX.forwardVelocity(600, -150);

    outtake(1000);

    runIntake(600);
    fullChassis.insert({ 10, -38, convertToRadians(300) }, 500);
    fullChassis.insert({2.5, -36, convertToRadians(295)}, 700);
    p1 = { -2, -36.5 }; p2 = { -1, -38.5 }; p3 = { 18, -55.25 }; p4 = { 39.55, -55 };
    Spline spline2({ p1, p2, p3, p4 });
    fullChassis.insert(spline2, 30, 1100);
    chassisController.changeFloorVel(85);
    fullChassis.execute();
    runUptake(15);

    //GOAL 4
    scoreBalls(2, 1, 1000);
    pros::delay(100);
    newX.forwardVelocity(500, -125);

    chassisController.toAngle(convertToRadians(22));
    outtake(1000);
    pros::delay(100);

    turnCont.setGains(turnless);

    runIntake(600);
    p1 = { 37, -60 }; p2 = { 35, -67 }; p3 = { 33, -74 }; p4 = { 33, -82.5 };
    Spline spline3({ p1, p2, p3, p4 });
    fullChassis.insert(spline3, 30, 1200);
    fullChassis.execute();

    pros::delay(350);

    turnCont.setGains(skillsTurn3);
    fullChassis.insert({43.5, -86.5, convertToRadians(0)}, 700);
    fullChassis.execute();

    runIntake(0);
    newX.forwardVelocity(400, 150);

    pros::delay(150);

    odomSys.setState({ 45, -78.6, 0 });

    newX.forwardVelocity(200, -150);

    fullChassis.insert({ 40, -88.25, convertToRadians(315) }, 500);
    fullChassis.insert({ 46.25, -94, convertToRadians(315) }, 50);
    fullChassis.execute();

    //GOAL 1
    scoreBallsSkills(0, 3, 1000);
    pros::delay(100);

    newX.forwardVelocity(500, -125);

    driveCont.setGains(skillsStraight2);
    turnCont.setGains(skillsTurn2);

    fullChassis.insert({ 0, -82.75, convertToRadians(270) }, 2000);
    fullChassis.insert({ 0, -91.75, convertToRadians(270) }, 200);
    fullChassis.execute();

    //GOAL 2
    scoreBallsSkills(0, 1, 1000);
    pros::delay(100);

    phase=1;
    fullChassis.insert({ 29, -85, convertToRadians(270)}, 500);
    fullChassis.insert({ 30, -80, convertToRadians(245)}, 500);
    fullChassis.insert({ 26.5, -65, convertToRadians(220)}, 500);
    fullChassis.execute();
    runIntake(600);
    pros::delay(250);

    p1 = { 14, -64 }; p2 = { -4, -75 }; p3 = { -29, -100 }; p4 = { -39, -110 };
    Spline spline4({ p1, p2, p3, p4 });
    fullChassis.insert(spline4, 35, 2000);
    fullChassis.insert({-43.5, -108.25, convertToRadians(225) }, 200);
    chassisController.changeFloorVel(75);
    fullChassis.execute();

    pros::delay(100);

    //GOAL 3
    scoreBalls(1, 2, 1000);
    shoot(1);
    pros::delay(100);
    newX.forwardVelocity(400, -200);

    outtake(1000);
    pros::delay(100);

    runIntake(600);
    fullChassis.insert({-45.5, -89, convertToRadians(180) }, 1000);

    newX.forwardVelocity(150, 75);
    
    driveCont.setGains(skillsStraight3);
    turnCont.setGains(skillsTurn3);

    chassisController.changeFloorVel(50);

    odomSys.setState({ -46, -89, convertToRadians(180) });
    fullChassis.insert({ -30, -89, convertToRadians(135) }, 300);
    fullChassis.insert({-39, -70, convertToRadians(90) }, 500);
    fullChassis.execute();

    driveCont.setGains(straight);

    fullChassis.insert({-37.85, -53.75, convertToRadians(180) }, 500);
    fullChassis.execute();
    runUptake(15);

    //GOAL 6
    scoreBalls(2, 2, 1000);
    pros::delay(100);

    newX.forwardVelocity(100, -100);

    fullChassis.insert({-33.5, -48.5, convertToRadians(225) }, 500);
    fullChassis.execute();

    outtake(1000);
    pros::delay(250);
    runIntake(600);

    chassisController.toAngle(0);

    pros::delay(250);
    fullChassis.insert({-24.5, -51.25, convertToRadians(5) }, 500);
    fullChassis.execute();

    index();

    //MID GOAL
    runIntake(300);
    newX.forwardVelocity(750, 100);

    shoot(1);

    runUptake(15);
    runIntake(450);
    pros::delay(1000);
    
    runIntake(-150);
    newX.forwardVelocity(650, -200);

    odomSys.setState({ -35, -50.5, 0 });

    chassisController.toAngle(convertToRadians(330));

    outtake(1750);
    pros::delay(250);
    runIntake(600);

    p1 = { -35, -47 }; p2 = { -10, -38 }; p3 = { -7.5, -40 }; p4 = { -8, -12.5 };
    Spline spline5({ p1, p2, p3, p4 });
    fullChassis.insert(spline5, 35, 1500);
    fullChassis.execute();

    newX.forwardVelocity(500, 150);
    //GOAL 8
    scoreBalls(1, 1, 1000);
    pros::delay(100);
    newX.forwardVelocity(500, -125);

    chassisController.toAngle(convertToRadians(75));
    outtake(1000);
    pros::delay(100);
    runIntake(600);

    chassisController.toAngle(convertToRadians(180));

    fullChassis.insert({ -30, -10, convertToRadians(180) }, 700);
    p1 = { -30, -10 }; p2 = { -40, -10 }; p3 = { -40, -22 }; p4 = { -52.5, -22 };
    Spline spline6({ p1, p2, p3, p4 });
    fullChassis.insert(spline6, 30, 700);
    fullChassis.execute();

    fullChassis.insert({ -50.75, -17.86, convertToRadians(135) }, 700);
    fullChassis.execute();

    newX.forwardVelocity(700, 200);
    scoreBalls(2, 2);

    newX.forwardVelocity(300, -150);
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

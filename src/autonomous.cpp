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

pros::ADIEncoder rightEnc(RightEncTop, RightEncBot, true);
pros::ADIEncoder leftEnc(LeftEncTop, LeftEncBot, true);
pros::ADIEncoder horEnc(HorEncTop, HorEncBot, true);

Chassis newChassis{ 2.75, 12.75, 0.5 };
Sensor_vals valStorage{ 0, 0, 0, true };

ThreeTrackerOdom odomSys(newChassis);

PIDConsts straight{12, 0, 0, 0 };
PIDConsts straight2{ 23, 0, .1, 0 };
PIDConsts straight3{ 18.5, 0, .02, 0 };
PIDConsts turn{ 110, 0, 0, 0 };
PIDConsts turn2{ 200, 0, .15, 0 };
PIDConsts skillsStartStraight{ 6, 0, 0, 0 };
PIDConsts skillsStartTurn{ 90, 0, 0, 0 };
PIDConsts hardTurn{ 275, 0, 0, 0 };

PIDController driveCont(straight);
PIDController turnCont(turn);

XDrive newX({ FrontRightWheelPort, BackRightWheelPort }, { -FrontLeftWheelPort, -BackLeftWheelPort });

PursuitController chassisController(&newX, &odomSys, &driveCont, &turnCont);

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


double convertToRadians(double input) {
    return input / 180 * 3.14159;
}

int ballsIn = 0;

void ballCounter(void* p) {
    double lastVal = detectLimit + 50;
    double curVal = 0;
    while (true) {
        curVal = botDistance.get();
        if (curVal < detectLimit && lastVal > detectLimit)
            ballsIn++;
        lastVal = curVal;

        pros::delay(20);
    }
}
void Scoreballs(int balls, int delay=1500){
  runIntake(300);
  runUptake(600);
  double initTime = pros::millis();
  while (ballsIn < balls && (pros::millis() - initTime) < delay) {
      pros::delay(20);
  }
  runIntake(0);
  pros::delay(delay);
  runUptake(0);
}
void subsystemSynchronous(void* p) {
    rightUptake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    leftUptake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rightIntake.set_brake_mode(MOTOR_BRAKE_BRAKE);
    leftIntake.set_brake_mode(MOTOR_BRAKE_BRAKE);

    flipOut();
    pros::delay(500);
    runUptake(-300);
    pros::delay(250);
    runUptake(0);

    /*
    pros::delay(500);
    notification.take(TIMEOUT_MAX);
    double initTime;

    Scoreballs(1,100);
    notification.give();
    pros::delay(500);
    notification.take(TIMEOUT_MAX);
    Scoreballs(2,175);

    notification.give();
    pros::delay(500);
    notification.take(TIMEOUT_MAX);
    Scoreballs(2,80);
    notification.give();
    pros::delay(500);
    notification.take(TIMEOUT_MAX);
    Scoreballs(2,450);
    notification.give();
    */
    
    pros::delay(500);
    notification.take(TIMEOUT_MAX);
    runIntake(200);
    pros::delay(750);
    runUptake(600);
    runIntake(-200);
    runUptake(0);
    notification.give();

    pros::delay(500);
    notification.take(TIMEOUT_MAX);
    runIntake(200);
    pros::delay(650);
    runUptake(600);
    pros::delay(800);
    runIntake(0);
    runUptake(0);
    notification.give();

    pros::delay(500);
    notification.take(TIMEOUT_MAX);
    runIntake(200);
    pros::delay(750);
    runUptake(600);
    pros::delay(1250);
    runIntake(0);
    runUptake(0);
    notification.give();
}

void robotTask(void* p) {
    newX.changeGearset(pros::E_MOTOR_GEARSET_06);

  //skills start
    /*
    notification.take(0);
    driveCont.setGains(skillsStartStraight);
    turnCont.setGains(skillsStartTurn);
    State targetState{ 15, -23 , convertToRadians(70) };
    chassisController.toPoint(targetState);
    pros::delay(10);
    chassisController.toPoint({ 27 , -10 , convertToRadians(90) });
    pros::delay(10);
    notification.give();
    newX.forwardVelocity(450, 150); //first goal
    pros::delay(100);
    notification.take(TIMEOUT_MAX);
    pros::delay(150);
    newX.forwardVelocity(450, -200);
    runIntake(-200);
    runUptake(-600);
    pros::delay(850);
    runIntake(200);
    runUptake(0);
    chassisController.toPoint({ 38.5 , -4 , convertToRadians(3) });
    pros::delay(250);
    chassisController.toPoint({ 77 , -11.5 , convertToRadians(5) });
    runIntake(300);
    index();
    pros::delay(350);
    runUptake(0);
    runIntake(0);
    chassisController.toPoint({ 73 , -0.5 , convertToRadians(47) });
    newX.forwardVelocity(500,75); // second goal
    ballsIn = 0;
    notification.give();
    pros::delay(450);
    // newX.forwardVelocity(150, -100);
    pros::delay(150);
    // newX.forwardVelocity(350, 125);
    notification.take(TIMEOUT_MAX);
    newX.forwardVelocity(500, -150);
    runIntake(-200);
    runUptake(-600);
    pros::delay(800);
    runUptake(0);
    chassisController.toPoint({53,-20,convertToRadians(287)});
    runIntake(600);
    runUptake(100);
    pros::delay(10);
    newX.forwardVelocity(600,150);
    pros::delay(150);
    runUptake(0);
    chassisController.toAngle(convertToRadians(12));
    chassisController.toPoint({67,-45.25,convertToRadians(12)});
    ballsIn = 0;
    newX.forwardVelocity(500, 150);
    pros::delay(10);
    newX.forwardVelocity(750,175); // third goal
    pros::delay(10);
    notification.give();
    pros::delay(150);
    notification.take(TIMEOUT_MAX);
    newX.forwardVelocity(400,-200);
    pros::delay(10);
    runUptake(-600);
    runIntake(-450);
    pros::delay(400);
    runUptake(0);
    runIntake(0);
    chassisController.toPoint({80,-78.75, convertToRadians(15)});
    pros::delay(10);
    runIntake(600);
    newX.forwardVelocity(750,200);
    State curState = odomSys.getState();
    curState.theta = 0;
    odomSys.setState(curState);
    pros::delay(300);
    newX.forwardVelocity(325,-200);
    pros::delay(10);
    chassisController.toPoint({91.5,-85.5,convertToRadians(320)});
    runIntake(200);
    newX.forwardVelocity(350,200);
    ballsIn=0;
    notification.give();
    pros::delay(500);
    notification.take(TIMEOUT_MAX);
    pros::delay(850);
    runUptake(0);
    runIntake(0);
    newX.forwardVelocity(200,-200);
    */
    // +6,-11.5

    /*
    double startTime = pros::millis();
    notification.take(0);
    chassisController.toPoint({ 14, -13, convertToRadians(45) });
    pros::delay(100);
    notification.give();
    newX.forwardVelocity(975, 100);
    notification.take(TIMEOUT_MAX);
    pros::delay(150);
    newX.forwardVelocity(650, -125);
    runIntake(250);
    pros::delay(150);
    runUptake(300);
    runIntake(-200);
    pros::delay(750);
    runUptake(0);
    runIntake(300);
    chassisController.toPoint({ 14, -20, convertToRadians(273)});
    chassisController.toPoint({ 14.5, -30, convertToRadians(273) });
    newX.forwardVelocity(450, 75);
    chassisController.toPoint({ 0, -39, convertToRadians(280) });
    runIntake(0);
    newX.strafeVelocity(500, 75);
    pros::delay(350);
    chassisController.toPoint({ 2, -29, convertToRadians(240) });
    newX.forwardVelocity(950, 100);
    runUptake(600);
    pros::delay(750);
    runUptake(0);
    while (pros::millis() - startTime < 14950) {
        pros::delay(10);
    }
    newX.forwardVelocity(50, -100);
    */




    notification.take(0);
    pros::delay(250);
    State targetState{ 14.5, -12.5 , convertToRadians(45) };
    chassisController.toPoint(targetState);
    pros::delay(100);
    ballsIn = 0;
    notification.give();
    newX.forwardVelocity(975, 220);
    newX.forwardVelocity(200, -200);
    pros::delay(100);
    newX.forwardVelocity(250, 200);
    notification.take(TIMEOUT_MAX);
    driveCont.setGains(straight);
    pros::delay(150);
    newX.forwardVelocity(650, -125);
    runUptake(300);
    runIntake(-200);
    pros::delay(100);
    runUptake(0);
    runIntake(0);
    chassisController.toAngle(convertToRadians(270));

    /*
    driveCont.setGains(straight2);
    turnCont.setGains(turn2);
    chassisController.toPoint({ -23.5 , 20 , convertToRadians(270) });
    runUptake(0);
    runIntake(0);
    ballsIn = 0;
    notification.give();
    newX.forwardVelocity(850, 170);
    newX.forwardVelocity(200, -200);
    newX.forwardVelocity(250, 220);
    notification.take(TIMEOUT_MAX);
    runIntake(600);
    runUptake(600);
    newX.forwardVelocity(370, -200);
    runIntake(-350);
    runUptake(-350);
    pros::delay(250);
    
    chassisController.toPoint({ -29.5 , 38.5 , convertToRadians(220) });
    runIntake(0);
    runUptake(0);
    pros::delay(50);
    driveCont.setGains(straight3);
    chassisController.toPoint({ -67  , 14 , convertToRadians(215) });
    ballsIn = 0;
    notification.give();
    newX.forwardVelocity(1050, 220);
    pros::delay(100);
    notification.take(TIMEOUT_MAX);
    runUptake(0);
    newX.forwardVelocity(300, -200);
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

        std::string text = "Balls: " + std::to_string(ballsIn);
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

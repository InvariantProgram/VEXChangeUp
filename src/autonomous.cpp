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

  Chassis newChassis{ 2.75, 13.4, 6.25 };
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

  void scoreIntakes(int inBalls, int delay=2500) {
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


  void score3Balls(int delay=2500) {
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

  void scoreBalls(int shootBalls, int inBalls, int delay=2500) {
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
    index();
    pros::delay(50);
    runIntake(-5);

    delayUntilPhase(2);
    pros::delay(350);
    outtake(1350);
    pros::delay(50);
    runIntake(600);
    runUptake(30);

    delayUntilPhase(3);
    pros::delay(500);
    outtake(650);
    pros::delay(300);
    runIntake(600);

    delayUntilPhase(4);
    pros::delay(2750);
    index();
    runUptake(-35);
    pros::delay(275);
    runUptake(-5);


    delayUntilPhase(5);
    runIntake(-200);
    pros::delay(150);
    runIntake(0);
    pros::delay(325);
    runUptake(-200);
    runIntake(-600);
    pros::delay(225);
    runUptake(0);
    pros::delay(500);
    runIntake(0);


    delayUntilPhase(6);
    pros::delay(200);
    outtake(600);
    pros::delay(100);
    runIntake(600);

    delayUntilPhase(7);
    pros::delay(1500);
    index();
    pros::delay(50);
    runIntake(600);

    delayUntilPhase(8);
    pros::delay(350);
    outtake(1000);
    pros::delay(100);
    runIntake(600);
    runUptake(30);
    pros::delay(2750);
    runIntake(0);

    delayUntilPhase(9);
    pros::delay(500);
    outtake(700);
    pros::delay(100);
    runIntake(600);

    delayUntilPhase(10);
    pros::delay(2750);
    index();

    delayUntilPhase(11);
    runIntake(-200);
    pros::delay(150);
    runIntake(0);
    pros::delay(325);
    runUptake(-200);
    runIntake(-600);
    pros::delay(200);
    runUptake(0);
    pros::delay(500);
    runIntake(0);

    delayUntilPhase(12);
    pros::delay(300);
    outtake(750);
    pros::delay(50);
    runIntake(600);

    delayUntilPhase(13);
    index();
    pros::delay(300);
    runIntake(-600);


/*
    delayUntilPhase(6);
    pros::delay(500);
    outtake(650);
    pros::delay(100);
    runIntake(600);
    delayUntilPhase(7);
    pros::delay(325);
    runUptake(-200);
    runIntake(-600);
    pros::delay(200);
    runUptake(0);
    pros::delay(500);
    runIntake(0);
    delayUntilPhase(8);
    pros::delay(200);
    outtake(850);
    pros::delay(100);
    runIntake(600);
*/

  }

  /*
  * 7: Score3Balls OR scoreBalls(3, 2) - meh?
  * 4: scoreIntakes(1)
  * 1: scoreBalls(1, 2)
  * 2: scoreBalls(1, 1)
  */


  void robotTask(void* p) {
      Point p1, p2, p3, p4;
      newX.changeGearset(pros::E_MOTOR_GEARSET_06);

      double startTime = pros::millis();

      driveCont.setGains({ 21, 0, 0.002, 0 });
      turnCont.setGains({ 205, 0, 0, 0 });
      chassisController.changeFloorVel(25);

      fullChassis.changeError(0.25);

      pros::delay(675);

      runIntake(600);

      p1 = { 0,0 }, p2 = { 10, 0 }, p3 = { 38, -17 }, p4 = { 44.5, -17 };
      Spline spline1({ p1, p2, p3, p4 });
      fullChassis.insert(spline1, 30, 1250);
      fullChassis.execute();

      phase = 1;
      driveCont.setGains({ 16.75, 0, 0.01, 0 }); //17.5
      turnCont.setGains({ 280, 0, .25, 0 });
      fullChassis.insert({ 40.25, -1.5, convertToRadians(42) }, 100);
      fullChassis.execute();
      runIntake(0);
      newX.runMotors(slowerForward);
      pros::delay(450);
      //GOAL 7
      score3Balls(1000); //1200
      pros::delay(250);
      newX.stop(true);
      pros::delay(150);
      odomSys.setState({0,0,convertToRadians(0)});
      pros::delay(150);

      phase = 2;
      driveCont.setGains({ 21, 0, 0.035, 0 }); //19
      fullChassis.insert({ -18, 0, convertToRadians(30) }, 600);
      fullChassis.insert({ -58.25, 2, convertToRadians(250) }, 1275);
      p1 = { -60, 0 }, p2 = { -67.5, -23 }, p3 = { -53.25, -26.25 }, p4 = { -43, -34.75 };
      Spline seventofour({ p1, p2, p3, p4 });
      fullChassis.insert(seventofour, 45, 1750);
      fullChassis.execute();

      runIntake(-5);
      newX.runMotors(slowerForward);
      pros::delay(625);
      newX.runMotors(maxSpeed);
      //GOAL 4
      scoreIntakes(1);
      newX.stop(true);
      pros::delay(350);
      odomSys.setState({0,0,convertToRadians(0)});
      pros::delay(350);


      phase = 3;
      driveCont.setGains({ 14.75, 0, 0.0002, 0 });
      fullChassis.insert({ -8, 0, 0} , 400);
      fullChassis.insert({ -8, -8, convertToRadians(50) }, 300);
      fullChassis.execute();

      phase = 4;
      driveCont.setGains({17, 0, 0.00075, 0 });
      turnCont.setGains({ 280, 0, .2, 0 });
      fullChassis.insert({ -15.25, -32, convertToRadians(270)}, 1000);
      fullChassis.insert({ -14.75, -46.5, convertToRadians(270) }, 350);
      fullChassis.insert({ 3.55, -35.25, convertToRadians(0)}, 500);
      fullChassis.execute();

      driveCont.setGains({16.25, 0, 0.001, 0 }); //17
      turnCont.setGains({ 260, 0, .15, 0 });
      fullChassis.insert({ -10, -46.5, convertToRadians(330) }, 400);
      fullChassis.insert({ 0.75, -51.85, convertToRadians(320) }, 100);
      fullChassis.execute();

      newX.runMotors(slowerForward);
      pros::delay(600);
      //GOAL 1
      scoreBalls(1, 2, 750);
      newX.stop(true);
      pros::delay(150);
      odomSys.setState({0,0,convertToRadians(0)});
      pros::delay(150);

      phase = 5;
      newX.forwardVelocity(350, -300);
      driveCont.setGains({20.75, 0, 0.0002, 0 });
      turnCont.setGains({ 285, 0, .2, 0 });
      fullChassis.insert({-18,  0, convertToRadians(0)}, 200);
      fullChassis.insert({ -40, -27, convertToRadians(340)}, 300);
      fullChassis.insert({ -52, -30, convertToRadians(320) }, 300);
      fullChassis.insert({ -47, -33.5 , convertToRadians(315)}, 400);
      fullChassis.insert({ -44.15, -35.25, convertToRadians(315) }, 200);
      fullChassis.execute();

      newX.runMotors(maxSpeed);
      //GOAL 2
      runUptake(300);
      runIntake(300);
      pros::delay(475);
      runUptake(0);
      runIntake(0);
      newX.stop(true);
      pros::delay(350);
      odomSys.setState({0,0,convertToRadians(0)});
      pros::delay(250);

      phase = 6;
      driveCont.setGains({ 20.75, 0, 0.0002, 0 });
      turnCont.setGains({ 300, 0, .2, 0 });
      fullChassis.insert({ -5.5, 2, convertToRadians(35) }, 450);
      fullChassis.insert({ -10, 0, convertToRadians(100) }, 300);
      fullChassis.insert({ -16, -1, convertToRadians(160) }, 300);
      fullChassis.insert({ -23.5, 0.1, convertToRadians(180) }, 200);
      fullChassis.execute();

      phase = 7;
      turnCont.setGains({ 275, 0, .2, 0 });
      fullChassis.insert({-17, -21, convertToRadians(310)}, 750);
      fullChassis.insert({-4, -37 , convertToRadians(295)}, 750);
      fullChassis.insert({-13.5, -45, convertToRadians(285)}, 400);
      fullChassis.insert({-16.5, -55, convertToRadians(270)}, 1250);
      fullChassis.insert({ -5, -45, convertToRadians(315) }, 300);
      fullChassis.insert({ -2, -50, convertToRadians(315) }, 200);
      fullChassis.insert({ -0.5, -52.75, convertToRadians(315)}, 200);
      fullChassis.execute();

      runIntake(-5);
      newX.runMotors(slowerForward);
      pros::delay(400);
      //GOAL 3
      score3Balls(1200);
      pros::delay(150);
      newX.stop(true);
      pros::delay(150);
      odomSys.setState({0,0,convertToRadians(0)});
      pros::delay(150);


      phase = 8;
      driveCont.setGains({ 21, 0, 0.0035, 0 });
      turnCont.setGains({ 180, 0, .2, 0 });
      fullChassis.insert({-18, 0, convertToRadians(30)}, 150);
      fullChassis.insert({ -61, 5, convertToRadians(250) }, 1200);
      p1 = { -60.5, 0 }, p2 = { -72.5, -23 }, p3 = { -53.25, -26.15 }, p4 = { -43, -35 };
      Spline threetosix({ p1, p2, p3, p4 });
      fullChassis.insert(threetosix, 45, 1750);
      fullChassis.execute();

      runIntake(-5);
      newX.runMotors(slowerForward);
      pros::delay(450);
      newX.runMotors(maxSpeed);
      //GOAL 6
      scoreIntakes(1);
      newX.stop(true);
      pros::delay(350);
      odomSys.setState({0,0,convertToRadians(0)});
      pros::delay(250);

      driveCont.setGains({ 17, 0, 0.0002, 0 });

      phase = 9;
      fullChassis.insert({ -8, 0, 0} , 400);
      fullChassis.insert({ -8, -8, convertToRadians(50) }, 300);
      fullChassis.execute();

      phase = 10;
      driveCont.setGains({20, 0, 0.00075, 0 });
      turnCont.setGains({ 267.5, 0, .2, 0 });
      fullChassis.insert({ -15, -30, convertToRadians(270)}, 1000);
      fullChassis.insert({ -14, -45, convertToRadians(270) }, 350);
      fullChassis.insert({ -10, -35.25, convertToRadians(330) }, 400);
      fullChassis.insert({ 3.1, -35.25, convertToRadians(0)}, 200);
      fullChassis.execute();

      driveCont.setGains({14.75, 0, 0.001, 0 });
      turnCont.setGains({ 267.5, 0, .15, 0 });
      fullChassis.insert({ -13, -46.5, convertToRadians(330) }, 400);
      fullChassis.insert({ 0, -51, convertToRadians(315) }, 500);
      fullChassis.execute();

      newX.runMotors(forwardVel);
      pros::delay(475);
      newX.runMotors(maxSpeed);
      pros::delay(85);
      //GOAL 9
      scoreBalls(1, 2, 750);
      pros::delay(250);
      newX.stop(true);
      pros::delay(150);
      odomSys.setState({0,0,convertToRadians(0)});
      pros::delay(150);


      phase = 11;
      newX.forwardVelocity(350, -300);
      driveCont.setGains({ 21, 0, 0.0002, 0 });
      turnCont.setGains({ 300, 0, .2, 0 });
      fullChassis.insert({ -18,  0, convertToRadians(0) }, 200);
      fullChassis.insert({ -40, -27, convertToRadians(340) }, 300);
      fullChassis.insert({ -52, -30, convertToRadians(320) }, 300);
      fullChassis.insert({ -47, -33.5 , convertToRadians(315) }, 400);
      fullChassis.insert({ -45, -34.5, convertToRadians(315) }, 200);
      fullChassis.execute();


      newX.runMotors(slowerForward);
      pros::delay(350);
      newX.runMotors(maxSpeed);
      pros::delay(200);
      //GOAL 8
      scoreBalls(1, 2, 450);
      newX.stop(true);
      pros::delay(350);
      odomSys.setState({0,0,convertToRadians(0)});
      pros::delay(350);

      phase = 12;
      driveCont.setGains({20.5, 0, 0.0002, 0});
      turnCont.setGains({ 275, 0, 0.0001, 0 });
      fullChassis.insert({ -2, 0, convertToRadians(0) }, 200);
      fullChassis.insert({-5.5, 0, convertToRadians(35)}, 100);
      fullChassis.execute();

      driveCont.setGains({ 20, 0, 0.004, 0 });
      fullChassis.insert({ -19, -0.25, convertToRadians(180) }, 500);
      fullChassis.insert({-25, 0.25, convertToRadians(180)}, 300);
      fullChassis.insert({ -32.5, -2, convertToRadians(180) }, 200);
      fullChassis.insert({ -35.25, -1.25, convertToRadians(180) }, 500);
      fullChassis.execute();

      runIntake(600);

      newX.runMotors(forwardVel);
      pros::delay(500);
      midSkills();

      phase = 13;
      newX.forwardVelocity(750, -450);
      runIntake(0);
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

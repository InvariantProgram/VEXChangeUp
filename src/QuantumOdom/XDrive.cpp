#include "QuantumOdom/XDrive.hpp"
#include <algorithm>

const double PI = 3.14159;

XDrive::XDrive() : rightMotorFront(pros::Motor(1)), rightMotorBack(pros::Motor(2)), leftMotorFront(pros::Motor(3)),
leftMotorBack(pros::Motor(4)) {
    errorBounds = 25;
    settleTime = 250;
}

XDrive::XDrive(ThreeTrackerOdom* iOdom, PIDController* iStraight, PIDController* iTurn, pros::ADIEncoder* iRightEnc, pros::ADIEncoder* iLeftEnc, pros::ADIEncoder* iStrafeEnc,
    std::array<int, 2> rightPorts, std::array<int, 2> leftPorts, int acceptableError, double timelimit) : odomObj(iOdom), driveCont(iStraight), turnCont(iTurn),
    rightEncoder(iRightEnc), leftEncoder(iLeftEnc), strafeEncoder(iStrafeEnc), rightMotorFront(pros::Motor(abs(rightPorts[0]))), rightMotorBack(pros::Motor(abs(rightPorts[1]))),
    leftMotorFront(pros::Motor(abs(leftPorts[0]))), leftMotorBack(pros::Motor(abs(leftPorts[1]))) {
    errorBounds = acceptableError;
    settleTime = timelimit;
    if (rightPorts[0] < 0)
        rightMotorFront.set_reversed(true);
    if (rightPorts[1] < 0)
        rightMotorBack.set_reversed(true);
    if (leftPorts[0] < 0)
        leftMotorFront.set_reversed(true);
    if (leftPorts[1] < 0)
        leftMotorBack.set_reversed(true);
}


void XDrive::setTimeLimit(double timelimit) {
    settleTime = timelimit;
}
void XDrive::setErrorBounds(int acceptableError) {
    errorBounds = acceptableError;
}

void XDrive::setParams(int acceptableError, double timelimit) {
    errorBounds = acceptableError;
    settleTime = timelimit;
}
void XDrive::driveDistance(double dist) {
    int withinCount = 0;
    bool isRunning = true;
    int currentReadings=0, unifiedOutput=0, rightOutput=0, leftOutput=0, rightVelocity=0, leftVelocity=0;
    printf("Target: %d\n", dist);
    driveCont->setTarget(dist);
    Point start{odomObj->getState().x, odomObj->getState().y};
    rightEncoder->reset(); leftEncoder->reset();
    while (isRunning) {
        currentReadings = OdomMath::computeDistance(start, odomObj->getState());
        printf("Current: %d\n", currentReadings);
        unifiedOutput = driveCont->step(currentReadings);
        rightOutput = unifiedOutput;
        leftOutput = unifiedOutput;

        rightMotorFront.move_velocity(rightOutput);
        leftMotorFront.move_velocity(leftOutput);
        rightMotorBack.move_velocity(rightOutput);
        leftMotorBack.move_velocity(leftOutput);
        
        if (abs(dist - currentReadings) < errorBounds)
            withinCount++;
        else
            withinCount = 0;
        if (withinCount * 20 >= settleTime) {
            isRunning = false;
            rightMotorFront.move_velocity(0);
            rightMotorBack.move_velocity(0);
            leftMotorFront.move_velocity(0);
            leftMotorBack.move_velocity(0);
        }
        std::array<int, 3> diffs = {leftEncoder->get_value(), rightEncoder->get_value(), strafeEncoder->get_value()};
        odomObj->odomStep(diffs);
        rightEncoder->reset(); leftEncoder->reset(); strafeEncoder->reset();
        pros::delay(20);
    }
}
void XDrive::drivePoint(const Point& iPoint) {
    turnPoint(iPoint);
    driveDistance(OdomMath::computeDistance(iPoint, odomObj->getState()));
}

void XDrive::turnPoint(const Point& iPoint) {
    bool isRunning = true;
    int withinCount = 0;
    int leftOutput, rightOutput, leftVelocity, rightVelocity;
    turnCont->setTarget(0);
    rightEncoder->reset(); leftEncoder->reset();
    int currentDifference;
    while (isRunning) {
        odomObj->odomStep({ leftEncoder->get_value(), rightEncoder->get_value(), strafeEncoder->get_value() });
        currentDifference = OdomMath::computeAngle(iPoint, odomObj->getState());
        rightOutput = turnCont->step(currentDifference);
        leftOutput = -1 * rightOutput;

        rightMotorFront.move_velocity(rightOutput);
        rightMotorBack.move_velocity(rightOutput);
        leftMotorBack.move_velocity(leftOutput);
        leftMotorFront.move_velocity(leftOutput);

        if (abs(currentDifference) * PI * odomObj->getChassis().width < errorBounds)
            withinCount++;
        else
            withinCount = 0;
        if (withinCount * 20 >= settleTime) {
            isRunning = false;
            rightMotorFront.move_velocity(0);
            rightMotorBack.move_velocity(0);
            leftMotorFront.move_velocity(0);
            leftMotorBack.move_velocity(0);
        }
        rightEncoder->reset(); leftEncoder->reset(); strafeEncoder->reset();
        pros::delay(20);
    }
}

void XDrive::runallMotors(int time, int speed) {
    rightMotorFront.move_velocity(speed);
    rightMotorBack.move_velocity(speed);
    leftMotorFront.move_velocity(speed);
    leftMotorBack.move_velocity(speed);
    pros::delay(time);
    rightMotorFront.move_velocity(0);
    rightMotorBack.move_velocity(0);
    leftMotorFront.move_velocity(0);
    leftMotorBack.move_velocity(0);
}
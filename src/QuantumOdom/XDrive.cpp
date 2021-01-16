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


double XDrive::angleClamp(double angle) {
    double result;
    int quotient = (int)(angle / (2 * PI));
    double remainder = angle - quotient * 2 * PI;
    if (remainder < 0) result = 2 * PI - remainder;
    else result = remainder;
    return result;
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
    double currentReadings=0, unifiedOutput=0, rightOutput=0, leftOutput=0, rightVelocity=0, leftVelocity=0;
    driveCont->setTarget(dist);
    Point start{odomObj->getState().x, odomObj->getState().y};
    while (isRunning) {
        currentReadings = OdomMath::computeDistance(start, odomObj->getState());
        unifiedOutput = driveCont->step(currentReadings);
        rightOutput = unifiedOutput;
        leftOutput = unifiedOutput;

        rightVelocity = rightMotorFront.get_actual_velocity();
        leftVelocity = leftMotorFront.get_actual_velocity();
        if (abs(rightVelocity - rightOutput) > 40)
            rightOutput = rightVelocity + 10 * (rightOutput - rightVelocity) / abs(rightOutput - rightVelocity);
        if (abs(leftVelocity - leftOutput) > 40)
            leftOutput = leftVelocity + 10 * (leftOutput - leftVelocity) / abs(leftOutput - leftVelocity);

        double sign = unifiedOutput / abs(unifiedOutput);
        rightMotorFront.move_velocity(sign * 50);
        leftMotorFront.move_velocity(sign * 50);
        rightMotorBack.move_velocity(sign * 50);
        leftMotorBack.move_velocity(sign * 50);
        
        if (abs(abs(dist) - currentReadings) < errorBounds)
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
        pros::delay(20);
    }
}
void XDrive::strafeDistance(double dist) {
    int withinCount = 0;
    bool isRunning = true;
    int currentReadings=0, unifiedOutput=0, topOutput=0, botOutput=0, topVelocity=0, botVelocity=0;
    driveCont->setTarget(dist);
    Point start{ odomObj->getState().x, odomObj->getState().y };
    while (isRunning) {
        currentReadings = OdomMath::computeDistance(start, odomObj->getState());
        printf("Diff: %i\n", currentReadings);
        unifiedOutput = driveCont->step(currentReadings);
        printf("Output: %i\n", unifiedOutput);
        topOutput = unifiedOutput;
        botOutput = unifiedOutput;

        rightMotorFront.move_velocity(-40);
        leftMotorFront.move_velocity(40);
        rightMotorBack.move_velocity(40);
        leftMotorBack.move_velocity(-40);

        if (abs(abs(dist) - currentReadings) < errorBounds)
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
    double leftOutput, rightOutput, leftVelocity, rightVelocity;
    turnCont->setTarget(0);
    double currentDifference;
    while (isRunning) {
        currentDifference = OdomMath::computeAngle(iPoint, odomObj->getState()) * odomObj->getChassis().width;
        if (currentDifference > PI) currentDifference -= 2 * PI;
        else if (currentDifference < -PI) currentDifference += 2 * PI;
        leftOutput = turnCont->step(currentDifference);
        rightOutput = -1 * leftOutput;

        rightMotorFront.move_velocity(rightOutput);
        rightMotorBack.move_velocity(rightOutput);
        leftMotorBack.move_velocity(leftOutput);
        leftMotorFront.move_velocity(leftOutput);

        if (abs(currentDifference) < errorBounds)
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
        pros::delay(20);
    }
}

void XDrive::turnAngle(double angle) {
    bool isRunning = true;
    int withinCount = 0;
    double leftOutput, rightOutput, leftVelocity, rightVelocity;
    double target = angle * PI / 180;

    turnCont->setTarget(0);
    double currentDifference;
    
    while (isRunning) {
        double diff = target - odomObj->getState().theta;
        if (diff > PI) currentDifference = diff - 2 * PI;
        else if (diff < -PI) currentDifference = diff + 2 * PI;
        else currentDifference = diff;
        currentDifference = currentDifference * odomObj->getChassis().width;

        leftOutput = turnCont->step(currentDifference);
        rightOutput = -1 * leftOutput;

        rightMotorFront.move_velocity(rightOutput);
        rightMotorBack.move_velocity(rightOutput);
        leftMotorBack.move_velocity(leftOutput);
        leftMotorFront.move_velocity(leftOutput);

        if (abs(currentDifference) < errorBounds)
            withinCount++;
        else
            withinCount = 0;
        if (withinCount * 20 >= settleTime) {
            isRunning = false;
            rightMotorFront.move_velocity(0);
            rightMotorBack.move_velocity(0);
            leftMotorFront.move_velocity(0);
            leftMotorBack.move_velocity(0);
            printf("Successful exit");
        }
        pros::delay(20);
    }
}

void XDrive::runallMotors(int time, int speed) {
    rightMotorFront.move(speed);
    rightMotorBack.move(speed);
    leftMotorFront.move(speed);
    leftMotorBack.move(speed);
    pros::delay(time);
    rightMotorFront.move_velocity(0);
    rightMotorBack.move_velocity(0);
    leftMotorFront.move_velocity(0);
    leftMotorBack.move_velocity(0);
}

void XDrive::strafeMotors(int time, int speed) {
    rightMotorFront.move(-speed);
    rightMotorBack.move(speed);
    leftMotorFront.move(speed);
    leftMotorBack.move(-speed);
    pros::delay(time);
    rightMotorFront.move_velocity(0);
    rightMotorBack.move_velocity(0);
    leftMotorFront.move_velocity(0);
    leftMotorBack.move_velocity(0);
}
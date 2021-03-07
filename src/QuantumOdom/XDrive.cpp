#include "QuantumOdom/XDrive.hpp"
#include <algorithm>

const double PI = 3.14159;

XDrive::XDrive() : rightMotorFront(pros::Motor(1)), rightMotorBack(pros::Motor(2)), leftMotorFront(pros::Motor(3)),
leftMotorBack(pros::Motor(4)) {
    errorBounds = 25;
    settleTime = 250;
}

XDrive::XDrive(ThreeTrackerOdom* iOdom, PIDController* iForward, PIDController* iTurn, PIDController* iStraight,
    pros::ADIEncoder* iRightEnc, pros::ADIEncoder* iLeftEnc, pros::ADIEncoder* iStrafeEnc,
    std::array<int, 2> rightPorts, std::array<int, 2> leftPorts,
    int acceptableError, double timelimit) : odomObj(iOdom), driveCont(iForward), turnCont(iTurn), straightCont(iStraight),
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

void XDrive::setSlew(int minDiff, int rate) {
    slewDiff = minDiff;
    slewrate = rate;
}

void XDrive::setParams(int acceptableError, double timelimit) {
    errorBounds = acceptableError;
    settleTime = timelimit;
}
void XDrive::driveDistance(double dist) {
    int withinCount = 0;
    bool isRunning = true;
    double currentReadings=0, unifiedOutput=0, rightOutput=0, leftOutput=0, rightVelocity=0, leftVelocity=0, straightenOutput=0;
    straightCont->setTarget(0);
    driveCont->setTarget(dist);
    Point start{odomObj->getState().x, odomObj->getState().y};
    double angle = odomObj->getState().theta;
    while (isRunning) {
        currentReadings = OdomMath::computeDistance(start, odomObj->getState());
        unifiedOutput = driveCont->step(currentReadings);
        if (abs(currentReadings - dist) > 5) straightenOutput = straightCont->step(angle - odomObj->getState().theta);
        else straightenOutput = 0;

        rightOutput = unifiedOutput + straightenOutput;
        leftOutput = unifiedOutput;

        rightVelocity = rightMotorFront.get_actual_velocity();
        if (abs(rightOutput - rightVelocity) > slewDiff) rightOutput = rightVelocity + slewrate * (rightOutput - rightVelocity) / abs(rightOutput - rightVelocity);
        leftVelocity = leftMotorFront.get_actual_velocity();
        if (abs(leftOutput - leftVelocity) > slewDiff) leftOutput = leftVelocity + slewrate * (leftOutput - leftVelocity) / abs(leftOutput - leftVelocity);

        rightMotorFront.move_velocity(rightOutput);
        leftMotorFront.move_velocity(leftOutput);
        rightMotorBack.move_velocity(rightOutput);
        leftMotorBack.move_velocity(leftOutput);
        
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
    int currentReadings=0, unifiedOutput=0, topOutput=0, botOutput=0, topVelocity=0, botVelocity=0, straightenOutput=0;
    straightCont->setTarget(0);
    driveCont->setTarget(dist);
    Point start{ odomObj->getState().x, odomObj->getState().y };
    double angle = odomObj->getState().theta;
    while (isRunning) {
        currentReadings = OdomMath::computeDistance(start, odomObj->getState());
        unifiedOutput = driveCont->step(currentReadings);
        straightenOutput = straightCont->step(angle - odomObj->getState().theta);
        topOutput = unifiedOutput;
        botOutput = (unifiedOutput > 0) ? unifiedOutput - straightenOutput : unifiedOutput + straightenOutput;

        topVelocity = rightMotorFront.get_actual_velocity();
        if (abs(topOutput - topVelocity) > slewDiff) topOutput = topVelocity + slewrate * (topOutput - topVelocity) / abs(topOutput - topVelocity);
        botVelocity = rightMotorBack.get_actual_velocity();
        if (abs(botOutput - botVelocity) > slewDiff) botOutput = botVelocity + slewrate * (botOutput - botVelocity) / abs(botOutput - botVelocity);

        rightMotorFront.move_velocity(-topOutput);
        leftMotorFront.move_velocity(topOutput);
        rightMotorBack.move_velocity(botOutput);
        leftMotorBack.move_velocity(-botOutput);

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

        leftOutput = turnCont->step(currentDifference);
        rightOutput = -1 * leftOutput;

        rightMotorFront.move_velocity(rightOutput);
        rightMotorBack.move_velocity(rightOutput);
        leftMotorBack.move_velocity(leftOutput);
        leftMotorFront.move_velocity(leftOutput);

        if (abs(currentDifference) < errorBounds / 4)
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
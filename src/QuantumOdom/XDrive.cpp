#include "QuantumOdom/XDrive.hpp"
#include <algorithm>

XDrive::XDrive() : rightMotorFront(pros::Motor(1)), rightMotorBack(pros::Motor(2)), leftMotorFront(pros::Motor(3)),
leftMotorBack(pros::Motor(4)) {}

XDrive::XDrive(std::array<int, 2> rightPorts, std::array<int, 2> leftPorts) : rightMotorFront(pros::Motor(abs(rightPorts[0]))), 
    rightMotorBack(pros::Motor(abs(rightPorts[1]))), leftMotorFront(pros::Motor(abs(leftPorts[0]))), 
    leftMotorBack(pros::Motor(abs(leftPorts[1]))) {
    if (rightPorts[0] < 0)
        rightMotorFront.set_reversed(true);
    if (rightPorts[1] < 0)
        rightMotorBack.set_reversed(true);
    if (leftPorts[0] < 0)
        leftMotorFront.set_reversed(true);
    if (leftPorts[1] < 0)
        leftMotorBack.set_reversed(true);
}

void XDrive::changeGearset(pros::motor_gearset_e_t gearset) {
    rightMotorFront.set_gearing(gearset);
    rightMotorBack.set_gearing(gearset);
    leftMotorFront.set_gearing(gearset);
    leftMotorBack.set_gearing(gearset);
}

void XDrive::changeBrakemode(pros::motor_brake_mode_e_t brake) {
    rightMotorFront.set_brake_mode(brake);
    rightMotorBack.set_brake_mode(brake);
    leftMotorFront.set_brake_mode(brake);
    leftMotorBack.set_brake_mode(brake);
}

void XDrive::stop(bool waitSettle = false) {
    rightMotorFront.move_velocity(0);
    rightMotorBack.move_velocity(0);
    leftMotorFront.move_velocity(0);
    leftMotorBack.move_velocity(0);
    if (waitSettle) {
        while (!(rightMotorFront.is_stopped() || rightMotorBack.is_stopped()
            || leftMotorFront.is_stopped() || leftMotorBack.is_stopped())) {
            pros::delay(10);
        }
    }
}

void XDrive::forwardVelocity(int time, int speed) {
    rightMotorFront.move_velocity(speed);
    rightMotorBack.move_velocity(speed);
    leftMotorFront.move_velocity(speed);
    leftMotorBack.move_velocity(speed);
    pros::delay(time);
    stop();
}

void XDrive::strafeVelocity(int time, int speed) {
    rightMotorFront.move_velocity(-speed);
    rightMotorBack.move_velocity(speed);
    leftMotorFront.move_velocity(speed);
    leftMotorBack.move_velocity(-speed);
    pros::delay(time);
    stop();
}

void XDrive::runMotors(std::array<int, 4> values) {
    rightMotorFront.move_velocity(values[0]);
    rightMotorBack.move_velocity(values[1]);
    leftMotorFront.move_velocity(values[2]);
    leftMotorBack.move_velocity(values[3]);
}

pros::motor_gearset_e_t XDrive::getGearset() {
    return rightMotorFront.get_gearing();
}
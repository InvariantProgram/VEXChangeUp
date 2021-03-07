#pragma once
#include "api.h"

class XDrive {
	private:
		pros::Motor rightMotorFront;
		pros::Motor rightMotorBack;
		pros::Motor leftMotorFront;
		pros::Motor leftMotorBack;
	public:
        //Default Constructor
        XDrive();
        /*
        * Initialize and set the motors of the X-drive
        * Assumes running all motors moves bot forwards
        * Set motorport values to negative for reverse
        */
        XDrive(std::array<int, 2> rightPorts, std::array<int, 2> leftPorts);
        /*
        * Sets the gearsets of the motors of the drive
        * @param gearset: New gearset to apply to the motors
        */
        void changeGearset(pros::motor_gearset_e_t gearset);
        /*
        * Sets the brakemode of the motors of the drive
        * @param brake: New brakemode to apply to the motors
        */
        void changeBrakemode(pros::motor_brake_mode_e_t brake);
        /*
        * Stops the robot by setting velocity of each motor to 0
        * @param waitSettle: Block execution until robot is fully stopped
        */
        void stop(bool waitSettle);
        /*
        * Drives forward with internal motor PID; input negative values to go backwards
        * @param time: duration of maneuver
        * @param speed: .move_velocity(speed)
        */
        void forwardVelocity(int time, int speed);
        /*
        * Strafes rightwards with internal motor PID; input negative values to go left
        * @param time: duration of maneuver
        * @param speed: .move_velocity(speed)
        */
        void strafeVelocity(int time, int speed);
        /*
        * Run motors at individual speeds
        * @param values: {rightFront, rightBack, leftFront, leftBack}
        */
        void runMotors(std::array<int, 4> values);
        /*
        * Run motors at individual speeds (converts double for easier usage)
        * @param values: {rightFront, rightBack, leftFront, leftBack}
        */
        void runMotors(std::array<double, 4> values);

        pros::motor_gearset_e_t getGearset();
};

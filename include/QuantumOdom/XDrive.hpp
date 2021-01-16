#pragma once
#include "api.h"
#include "QuantumOdom/OdomMath.hpp"
#include "QuantumOdom/PIDController.hpp"
#include "QuantumOdom/ThreeTrackerOdom.hpp"
#include "QuantumOdom/structDefs.hpp"

class XDrive {
	private:
		ThreeTrackerOdom* odomObj;

		PIDController* driveCont;
		PIDController* turnCont;
		int errorBounds;
		double settleTime;

		pros::ADIEncoder* rightEncoder;
		pros::ADIEncoder* leftEncoder;
		pros::ADIEncoder* strafeEncoder;

		pros::Motor rightMotorFront;
		pros::Motor rightMotorBack;
		pros::Motor leftMotorFront;
		pros::Motor leftMotorBack;

        double angleClamp(double angle);
	public:
        //Default Constructor
        XDrive();
        /*
        * Full constructor (No default constructor)
        * Assumes running all motors moves bot forwards
        * Set motorport values to negative for reverse
        * Acceptable Error in encoder ticks : Subject to change??
        * Settling time in milliseconds
        */
        XDrive(ThreeTrackerOdom* iOdom, PIDController* iStraight, PIDController* iTurn, pros::ADIEncoder* iRightEnc, 
            pros::ADIEncoder* iLeftEnc, pros::ADIEncoder* iStrafeEnc, 
            std::array<int, 2> rightPorts, std::array<int, 2> leftPorts, 
            int acceptableError, double timelimit);
        //Set the settle time for the drive - for the final section of the path
        void setTimeLimit(double timelimit);
        //Set settle range
        void setErrorBounds(int acceptableError);
        //Set settling parameters
        void setParams(int acceptableError, double timelimit);
        //Move forward until at a point
        void drivePoint(const Point& iPoint);
        //Drive forward a defined distance in inches
        void driveDistance(double dist);
        //Drive rightwards a defined distance in inches
        void strafeDistance(double dist);
        //Turns towards the point
        void turnPoint(const Point& iPoint);
        //Turns to a specified angle in degrees
        void turnAngle(double angle);
        //Runs motors for the specified time in milliseconds at the set speed
        void runallMotors(int time, int speed);
        //Strafes all motors for the specified time in milliseconds at the set speed
        void strafeMotors(int time, int speed);
};

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
	public:
        //Default Constructor
        XDrive();
        //Full constructor (No default constructor)
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
        //Dirve forward a defined distance
        void driveDistance(const double dist);
        //Turns towards the point
        void turnPoint(const Point& iPoint);
        //Runs motors for the specified time in milliseconds at the set speed
        void runallMotors(int time, int speed);

};

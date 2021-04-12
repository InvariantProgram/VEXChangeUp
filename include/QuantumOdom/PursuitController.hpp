#pragma once

#include "Matrix.hpp"
#include "OdomMath.hpp"
#include "PIDController.hpp"
#include "structDefs.hpp"
#include "ThreeTrackerOdom.hpp"
#include "XDrive.hpp"

#include <vector>
#include <math.h>
#include <array>
#include <algorithm>

class PursuitController {
	private:
		XDrive* chassis;
		ThreeTrackerOdom* odomSys;

		PIDController* distCont;
		PIDController* angleCont;

		double minVel = 100;

		double angleClamp(double input);

		static bool absComp(const double& a, const double& b);
	public:
		PursuitController(XDrive* iChassis, ThreeTrackerOdom* iOdom,
			PIDController* iForward, PIDController* iTurn);

		State getLocation();

		void resetPID();

		void changeFloorVel(double iSpeed);
		
		void impulsePoint(State newPoint);

		void toAngle(double newAngle);

		void stop();
};
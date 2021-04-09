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
		double errorBounds = 0.5;

		XDrive* chassis;
		ThreeTrackerOdom* odomSys;

		PIDController* distCont;
		PIDController* angleCont;

		double angleClamp(double input);

		static bool absComp(const double& a, const double& b);
	public:
		PursuitController(XDrive* iChassis, ThreeTrackerOdom* iOdom,
			PIDController* iForward, PIDController* iTurn);

		void toPoint(State newPoint);

		void toPointVel(State newPoint, double endVel);

		void toAngle(double newAngle);

		void stop();

		void changeError(double iError);
};
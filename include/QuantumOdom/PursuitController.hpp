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
	public:
		PursuitController(XDrive* iChassis, ThreeTrackerOdom* iOdom,
			PIDController* iForward, PIDController* iTurn);

		void toPoint(State newPoint, double speed, double slowRange);

		void changeError(double iError);
};
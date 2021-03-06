#pragma once

#include "Matrix.hpp"
#include "OdomMath.hpp"
#include "PIDController.hpp"
#include "structDefs.hpp"
#include "ThreeTrackerOdom.hpp"
#include "XDrive.hpp"

#include <vector>
#include <math.h>

class PursuitController {
	private:
		XDrive* chassis;
		ThreeTrackerOdom* odomSys;

		PIDController* distCont;
		PIDController* angleCont;
	public:
		PursuitController(XDrive* iChassis, ThreeTrackerOdom* iOdom, 
			PIDController* iForward, PIDController* iTurn);

		void toPoint(State newPoint);
};
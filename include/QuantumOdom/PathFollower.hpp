#pragma once

#include <vector>

#include "Matrix.hpp"
#include "OdomMath.hpp"
#include "PIDController.hpp"
#include "PursuitController.hpp"
#include "structDefs.hpp"
#include "ThreeTrackerOdom.hpp"
#include "XDrive.hpp"

class PathFollower {
	private:
		PursuitController chassisController;

		std::vector<Matrix> splines;
		std::vector<State> waypoints;
	public:
};
#pragma once

#include "Matrix.hpp"
#include "OdomMath.hpp"
#include "PIDController.hpp"
#include "structDefs.hpp"
#include "ThreeTrackerOdom.hpp"
#include "XDrive.hpp"

#include <vector>

class PursuitController {
	private:
		static Matrix Bezier = new Matrix({
			{-1, 3, -3, 1},
			{3, -6, 3, 0},
			{-3, 3, 0, 0},
			{1, 0, 0, 0}
			});
		static Matrix Hermite = new Matrix({
			{2, -2, 1, 1},
			{-3, 3, -2, -1},
			{0, 0, 1, 0},
			{1, 0, 0, 0}
			});

		std::vector<Matrix> splineList;
		std::vector<State> waypoints;

		XDrive* chassis;
		ThreeTrackerOdom* odomSys;

		PIDController* distCont;
		PIDController* angleCont;
	public:
		/*
		*/
};
#pragma once

#include <queue>
#include <tuple>

#include "Matrix.hpp"
#include "OdomMath.hpp"
#include "PIDController.hpp"
#include "PursuitController.hpp"
#include "structDefs.hpp"
#include "ThreeTrackerOdom.hpp"
#include "XDrive.hpp"
#include "Spline.hpp"

class PathFollower {
	private:
		PursuitController* chassisController;

		std::queue<State> waypoints;
		double distance;
		double velTarget;
	public:
		PathFollower(PursuitController* iCont);

		void insert(Spline iPath, int resolution);
		void insert(State iPoint);

		void changeError(double iError);
		void changeFloorVel(double speed);

		void logStates();

		void execute();
};
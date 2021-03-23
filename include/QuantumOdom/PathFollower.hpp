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

		std::queue<std::pair<State, bool>> waypoints;
	public:
		PathFollower(PIDController* iCont);

		void insert(Spline iPath, int resolution, bool doFinalSlow);
		void insert(State iPoint, bool doSlow);

		void execute();
};
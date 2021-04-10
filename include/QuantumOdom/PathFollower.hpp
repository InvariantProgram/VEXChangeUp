#pragma once

#include <queue>
#include <tuple>

#include "api.h"
#include "QuantumOdom/Matrix.hpp"
#include "QuantumOdom/OdomMath.hpp"
#include "QuantumOdom/PIDController.hpp"
#include "QuantumOdom/PursuitController.hpp"
#include "QuantumOdom/structDefs.hpp"
#include "QuantumOdom/ThreeTrackerOdom.hpp"
#include "QuantumOdom/XDrive.hpp"
#include "QuantumOdom/Spline.hpp"

class PathFollower {
	private:
		PursuitController* chassisController;

		std::queue<std::pair<State, int>> waypoints;

		double distance;

		double errorbounds = 0.5;
	public:
		PathFollower(PursuitController* iCont);

		void insert(Spline iPath, int resolution, int msComplete);
		void insert(State iPoint, int msComplete);

		void changeError(double iError);

		void logStates();

		void execute();
};
#include "QuantumOdom/PathFollower.hpp"

PathFollower::PathFollower(PIDController* iCont) : chassisController(iCont) { }


void PathFollower::insert(Spline iPath, int resolution) {
	for (int i = 0; i <= resolution; i++) {
		double input = (double)i / resolution;
		waypoints.push(iPath.getState(input));
	}
}

void PathFollower::insert(State iPoint) {
	waypoints.push(iPoint);
}

void PathFollower::execute() {
	while (!waypoints.empty()) {
		State iterTarget = waypoints.pop();
		chassisController->toPoint(iterTarget);
	}
}

void PathFollower::changeError(double iError) {
	chassisController->changeError(iError);
}
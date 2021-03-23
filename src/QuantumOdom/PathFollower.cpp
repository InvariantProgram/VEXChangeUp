#include "QuantumOdom/PathFollower.hpp"

PathFollower::PathFollower(PIDController* iCont) : chassisController(iCont) { }


void PathFollower::insert(Spline iPath, int resolution, bool doFinalSlow=true) {
	for (int i = 0; i < resolution; i++) {
		double input = (double)i / resolution;
		waypoints.push(std::make_pair(iPath.getState(input), false));
	}
	waypoints.push(std::make_pair(iPath.getState(1.0), doFinalSlow));
}

void PathFollower::insert(State iPoint, bool doSlow) {
	waypoints.push(std::make_pair(iPoint, doSlow));
}

void PathFollower::execute() {
	if (waypoints.empty()) return;
	else {
		State target = waypoints.pop();
		while (!waypoints.empty()) {
			
		}
	}
}
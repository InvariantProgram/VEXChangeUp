#include "QuantumOdom/PathFollower.hpp"


PathFollower::PathFollower(PursuitController* iCont) : chassisController(iCont) { distProp = 75; }

void PathFollower::insert(Spline iPath, int resolution) {
	for (int i = 0; i <= resolution; i++) {
		double input = (double)i / resolution;
		State newState = iPath.getState(input);
		if (!waypoints.empty()) 
			distance += OdomMath::computeDistance(waypoints.back(), newState);
		waypoints.push(newState);
	}
}

void PathFollower::insert(State iPoint) {
	if (!waypoints.empty()) distance += OdomMath::computeDistance(waypoints.back(), iPoint);
	waypoints.push(iPoint);
}

void PathFollower::execute() {
	while (!waypoints.empty()) {
		State iterTarget = waypoints.front();
		waypoints.pop();
		distance -= OdomMath::computeDistance(waypoints.front(), iterTarget);
		double targetEndVel = distance * distProp;
		chassisController->toPointVel(iterTarget, targetEndVel);
	}
}

void PathFollower::changeError(double iError) {
	chassisController->changeError(iError);
}

void PathFollower::changeDecrementProp(double kP) {
	distProp = kP;
}
#include "QuantumOdom/PathFollower.hpp"


PathFollower::PathFollower(PursuitController* iCont) : chassisController(iCont) { velTarget = 100; }

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
		chassisController->toPointVel(iterTarget, velTarget);
		distance -= OdomMath::computeDistance(waypoints.front(), iterTarget);
	}
	chassisController->stop();
}

void PathFollower::logStates() {
	std::queue<State> temp;
	while (!waypoints.empty()) {
		State current = waypoints.front();
		temp.push(current);
		printf("%f, %f, %f\n", current.x, current.y, current.theta);
		waypoints.pop();
	}
	printf("Distance: %f", distance);
	while (!temp.empty()) {
		waypoints.push(temp.front());
		temp.pop();
	}
}

void PathFollower::changeError(double iError) {
	chassisController->changeError(iError);
}

void PathFollower::changeFloorVel(double speed) {
	velTarget = speed;
}
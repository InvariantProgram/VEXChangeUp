#include "QuantumOdom/PathFollower.hpp"


PathFollower::PathFollower(PursuitController* iCont) : chassisController(iCont) {}

void PathFollower::insert(Spline iPath, int resolution, int msComplete) {
	int endTime = waypoints.back().second;
	for (int i = 0; i <= resolution; i++) {
		double input = (double)i / resolution;
		State newState = iPath.getState(input);
		int newTime = endTime + input * msComplete;
		if (!waypoints.empty()) 
			distance += OdomMath::computeDistance(waypoints.back().first, newState);
		waypoints.push(std::make_pair(newState, newTime));
	}
}

void PathFollower::insert(State iPoint, int msComplete) {
	int endTime = waypoints.back().second;
	if (!waypoints.empty()) distance += OdomMath::computeDistance(waypoints.back().first, iPoint);
	waypoints.push(std::make_pair(iPoint, endTime));
}

void PathFollower::execute() {
	chassisController->resetPID();

	while (!waypoints.empty()) {
		State iterTarget = waypoints.front().first;
		int iterTime = waypoints.front().second;
		waypoints.pop();
		do {
			chassisController->impulsePoint(iterTarget);
			pros::delay(20);
		} while (pros::millis() < iterTime);
		if (waypoints.empty()) {
			while (OdomMath::computeDistance(chassisController->getLocation(), iterTarget) > errorbounds) {
				chassisController->impulsePoint(iterTarget);
				pros::delay(20);
			}
		}
	}
	chassisController->stop();
}

void PathFollower::logStates() {
	std::queue<std::pair<State, int>> temp;
	while (!waypoints.empty()) {
		temp.push(waypoints.front());
		State current = waypoints.front().first;
		printf("%f, %f, %f", current.x, current.y, current.theta);
		printf(" | %f ms\n", waypoints.front().second);
		waypoints.pop();
	}
	printf("Distance: %f", distance);
	while (!temp.empty()) {
		waypoints.push(temp.front());
		temp.pop();
	}
}

void PathFollower::changeError(double iError) {
	errorbounds = iError;
}
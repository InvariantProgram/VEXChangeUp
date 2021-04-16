#include "QuantumOdom/PathFollower.hpp"


PathFollower::PathFollower(PursuitController* iCont) : chassisController(iCont) {}

void PathFollower::insert(Spline iPath, int resolution, int msComplete) {
	int endTime = 0;
	if (!waypoints.empty()) endTime = waypoints.back().second;
	for (int i = 0; i <= resolution; i++) {
		double input = (double)i / resolution;
		State newState = iPath.getState(input);
		int newTime = endTime + (double) (input * msComplete);
		if (!waypoints.empty()) 
			distance += OdomMath::computeDistance(waypoints.back().first, newState);
		waypoints.push(std::make_pair(newState, newTime));
	}
}

void PathFollower::insert(State iPoint, int msComplete) {
	int endTime = 0;
	if (!waypoints.empty()) endTime = waypoints.back().second;
	if (!waypoints.empty()) distance += OdomMath::computeDistance(waypoints.back().first, iPoint);
	waypoints.push(std::make_pair(iPoint, endTime + msComplete));
}

void PathFollower::execute() {
	double totalTime = waypoints.back().second + 2500;
	chassisController->resetPID();

	int startTime = pros::millis();
	while (!waypoints.empty()) {
		State iterTarget = waypoints.front().first;
		int iterTime = waypoints.front().second;
		waypoints.pop();
		do {
			chassisController->impulsePoint(iterTarget);
			pros::delay(20);
		} while (pros::millis() - startTime < iterTime);
		if (waypoints.empty()) {
			while (OdomMath::computeDistance(chassisController->getLocation(), iterTarget) > errorbounds) {
				chassisController->impulsePoint(iterTarget);
				pros::delay(10);
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
		printf(" | %i ms\n", waypoints.front().second);
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
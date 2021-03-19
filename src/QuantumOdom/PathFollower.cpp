#include "QuantumOdom/PathFollower.hpp"

PathFollower::PathFollower(PIDController* iCont) : chassisController(iCont) { }


void PathFollower::insert(Spline iPath, int resolution) {
	for (int i = 0; i < resolution; i++) {
		double input = (double)i / resolution;
		waypoints.push_back(std::make_pair(iPath.getState(input), false));
	}
	waypoints.push_back(std::make_pair(iPath.getState(1.0), true));
}

void PathFollower::insert(State iPoint, bool doSlow) {
	waypoints.push_back(std::make_pair(iPoint, doSlow));
}

void PathFollower::execute() {

}
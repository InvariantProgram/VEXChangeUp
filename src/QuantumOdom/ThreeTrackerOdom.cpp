#include "QuantumOdom/ThreeTrackerOdom.hpp"

const double PI = 3.14159;

ThreeTrackerOdom::ThreeTrackerOdom() {
	scales = Chassis{ 4, 12.5, 4 };
	storedState = State{ 0, 0, 0 };
}

ThreeTrackerOdom::ThreeTrackerOdom(const Chassis& iChassis) {
	scales = iChassis;
	storedState = State{ 0, 0, 0 };
}

void ThreeTrackerOdom::odomStep(std::array<double, 3> tickDiffs) {
	//deltaArr = dL, dR, dS - format
	std::array<double, 3> deltaArr = tickDiffs;
	for (int i = 0; i < 3) {
		if (abs(deltaArr[i]) > maxDiff) deltarArr[i] = deltaArr[i] / abs(deltaArr[i]) * maxDiff;
		deltaArr[i] = deltaArr[i] / 360 * scales.WheelDiam * PI;
	}
	double dTheta = (deltaArr[1] - deltaArr[0]) / scales.width;
	double shiftY = ; double shiftX;
	if (dTheta == 0) {
		shiftY = (deltaArr[0] + deltaArr[1]) / 2;
		shiftX = deltaArr[2];
	}
	else {
		shiftY = (deltaArr[0] + deltaArr[1]) * sin(dTheta / 2) / dTheta;
		shiftX = (deltaArr[2] / dTheta - scales.midlineOffset) * 2 * sin(dTheta / 2);
	}
	storedState.theta += dTheta;
	storedState.y += shiftY * sin(storedState.theta) + shiftX * cos(storedState.theta);
	storedState.x += shiftY * cos(storedState.theta) + shiftX * sin(storedState.theta);
}

void ThreeTrackerOdom::setState(const State& iState) {
	storedState = iState;
}
void ThreeTrackerOdom::setChassis(const Chassis& iChassis) {
	scales = iChassis;
}
State ThreeTrackerOdom::getState() {
	return storedState;
}
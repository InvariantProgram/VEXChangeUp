#include "QuantumOdom/ThreeTrackerOdom.hpp"
#include <math.h>

const double PI = 3.14159;

ThreeTrackerOdom::ThreeTrackerOdom() {
	scales = Chassis{ 4, 12.5, 4 };
	storedState = State{ 0, 0, 0 };
}

ThreeTrackerOdom::ThreeTrackerOdom(const Chassis& iChassis) {
	scales = iChassis;
	storedState = State{ 0, 0, 0 };
}

double ThreeTrackerOdom::angleClamp() {
	double result;
	int quotient = (int)(storedState.theta / (2 * PI));
	double remainder = storedState.theta - quotient * 2 * PI;
	if (remainder < 0) result = 2 * PI - remainder;
	else result = remainder;
	return result;
}

void ThreeTrackerOdom::odomStep(std::array<int, 3> tickDiffs) {
	//deltaArr = dL, dR, dS - format
	std::array<double, 3> deltaArr;
	for (int i = 0; i < 3; i++) {
		deltaArr[i] = tickDiffs[i];
	}
	for (int i = 0; i < 3; i++) {
		if (abs(deltaArr[i]) > maxDiff) deltaArr[i] = deltaArr[i] / abs(deltaArr[i]) * maxDiff;
		deltaArr[i] = deltaArr[i] / 360 * scales.WheelDiam * PI;
	}
	double dTheta = (deltaArr[1] - deltaArr[0]) / scales.width;
	double shiftY; double shiftX;
	if (dTheta == 0) {
		shiftX = (deltaArr[0] + deltaArr[1]) / 2;
		shiftY = deltaArr[2];
	}
	else {
		shiftX = (deltaArr[0] + deltaArr[1]) * sin(dTheta / 2) / dTheta;
		shiftY = (deltaArr[2] / dTheta - scales.midlineOffset) * 2 * sin(dTheta / 2);
	}
	storedState.theta += dTheta; storedState.theta = angleClamp();
	storedState.y += shiftX * sin(storedState.theta) + shiftY * cos(storedState.theta);
	storedState.x += shiftX * cos(storedState.theta) - shiftY * sin(storedState.theta);
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
Chassis ThreeTrackerOdom::getChassis() {
	return scales;
}
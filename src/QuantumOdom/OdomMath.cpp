#include "QuantumOdom\OdomMath.hpp"

const double PI = 3.14159;

double OdomMath::computeDistance(const Point& iPoint, const State& iState) {
	std::array<double, 2> diffs = computeDiffs(iPoint, iState);
	double squareSum = pow(diffs[0], 2) + pow(diffs[1], 2);
	return pow(squareSum, 0.5);
}

double OdomMath::computeDistance(const State& iState, const State& rhs) {
	Point convPoint{ rhs.x, rhs.y };
	return computeDistance(convPoint, iState);
}

double OdomMath::computeAngle(const Point& iPoint, const State& iState) {
	std::array<double, 2> diffs = computeDiffs(iPoint, iState);
	double principalAngle = atan2(diffs[1], diffs[0]);
	if (principalAngle < 0) principalAngle += 2 * PI;
	double diff = principalAngle - iState.theta;
	if (diff > PI) diff -= 2 * PI;
	else if (diff < -PI) diff += 2 * PI;
	return diff;
}

double OdomMath::computeAngle(const State& iState, const State& rhs) {
	double diff = rhs.theta - iState.theta;
	if (diff > PI) diff -= 2 * PI;
	else if (diff < -PI) diff += 2 * PI;
	return diff;
}

std::array<double, 2> OdomMath::computeAngleAndDistance(const Point& iPoint, const State& iState) {
	return {computeDistance(iPoint, iState), computeAngle(iPoint, iState)};
}

std::array<double, 2>  OdomMath::computeDiffs(const Point& iPoint, const State& iState) {
	return {iPoint.x - iState.x, iPoint.y - iState.y};
}

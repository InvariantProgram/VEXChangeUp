#include "QuantumOdom\OdomMath.hpp"

bool State::operator==(const State& rhs) const {
	return (x == rhs.x && y == rhs.y && theta == rhs.theta);
}
bool State::operator!=(const State& rhs) const {
	return !(rhs == *this);
}
State State::operator-(const State& rhs) {
	x -= rhs.x; y -= rhs.y; theta -= rhs.theta;
	return *this;
}
State State::operator+(const State& rhs) {
	x += rhs.x; y += rhs.y; theta += rhs.theta;
	return *this;
}


double OdomMath::computeDistance(const Point& iPoint, const State& iState) {
	std::array<double, 2> diffs = computeDiffs(iPoint, iState);
	double squareSum = pow(diffs[0], 2) + pow(diffs[1], 2);
	return pow(squareSum, 0.5);
}

double OdomMath::computeAngle(const Point& iPoint, const State& iState) {
	std::array<double, 2> diffs = computeDiffs(iPoint, iState);
	return atan2(diffs[1], diffs[0]);
}

std::array<double, 2> OdomMath::computeAngleAndDistance(const Point& iPoint, const State& iState) {
	return {computeDistance(iPoint, iState), computeAngle(iPoint, iState)};
}

std::array<double, 2>  OdomMath::computeDiffs(const Point& iPoint, const State& iState) {
	return {iPoint.x - iState.x, iPoint.y - iState.y};
}

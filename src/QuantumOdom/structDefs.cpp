#include "QuantumOdom/structDefs.hpp"

void Sensor_vals::setVals(int iRight, int iLeft, int iStrafe) {
    right = iRight;
    left = iLeft;
    middle = iStrafe;
	hasMiddle = true;
}
void Sensor_vals::setVals(int iRight, int iLeft) {
	right = iRight;
	left = iLeft;
	hasMiddle = false;
}

std::vector<double> Point::returnVals() {
	return { x, y };
}

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

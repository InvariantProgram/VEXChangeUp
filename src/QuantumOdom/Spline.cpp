#include "QuantumOdom/Spline.hpp"

Spline::Spline(std::array<Point, 4> iPoints) {
	std::vector<std::vector<double>> input;
	for (int i = 0; i < 4; i++) {
		std::vector<double> insert = { iPoints[i].x, iPoints[i].y };
		input.push_back(insert);
	}
	Matrix pMat(input);
	coeffs = Bezier * pMat;
}

State Spline::getState(double iTime) {
	std::vector<double> input;
	for (int i = 0; i < 4; i++) {
		input.push_back(pow(iTime, 3 - i));
	}
	Matrix potencies({ input });
	Matrix result = potencies * coeffs;

	double dy = 0;
	double dx = 0;
	for (int i = 0; i < coeffs.get_height(); i++) {
		dx += coeffs(i, 0) * potencies(0, i);
		dy += coeffs(i, 1) * potencies(0, i);
	}
	double theta = atan2(dy, dx);
	if (theta < 0) theta += 2 * 3.14159;

	State retState{ result(0, 0), result(0, 1), theta };
	return retState;
}
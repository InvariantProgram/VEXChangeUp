#pragma once

#include <array>
#include <vector>
#include <algorithm>
#include <math.h>

#include "Matrix.hpp"
#include "structDefs.hpp"

class Spline {
	private:
		static Matrix Bezier = new Matrix({
			{-1, 3, -3, 1},
			{3, -6, 3, 0},
			{-3, 3, 0, 0},
			{1, 0, 0, 0}
			});

		Matrix coeffs;
	public:
		Spline(std::array<Point, 4> iPoints);

		/*
		* Return the state associated on the spline at input time
		* @param iTime = Time (in seconds) to get state [Restricted from 0 to 1]
		* @return State with coordinates & direction = curvature
		*/
		State getState(double iTime);
};
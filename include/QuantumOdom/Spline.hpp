#pragma once

#include <array>
#include <vector>
#include <algorithm>
#include <math.h>

#include "Matrix.hpp"
#include "structDefs.hpp"

class Spline {
	private:
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
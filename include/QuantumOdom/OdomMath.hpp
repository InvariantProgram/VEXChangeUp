#pragma once
#include "QuantumOdom/structDefs.hpp"
#include <math.h>
#include <array>

class OdomMath {
	private:
		//Compute and return the difference in x and y, as a pair within an array in the same order
		static std::array<double, 2> computeDiffs(const Point& iPoint, const State& iState);
	public:
		/*
		* Compute the distance between a point and state
		* @param iPoint The point
		* @param iState The odom state
		* @return The distance between the state and the point
		*/
		static double computeDistance(const Point& iPoint, const State& iState);
		/*
		* Compute the angle between a point and odometry state
		* All operations in radians, constrained -pi to pi
		* @param iPoint the point
		* @param iState the odom state
		* @return The angle between odom state (includes direction) and the point
		*/
		static double computeAngle(const Point& iPoint, const State& iState);
		/*
		* Compute the angle between from a state to another state
		* All operations in radians, constrained -pi to pi
		* @param iState the initial odom state
		* @param rhs the final odom state
		* @return The angle between the two odom states
		*/
		static double computeAngle(const State& iState, const State& rhs);
		/*
		* Compute the angle and distance between a point and odometry state
		* All operations in radians, constrained -pi to pi
		* @param iPoint the point
		* @param iState the odom state
		* @return Pair, with (Distance, Angle)
		*/
		static std::array<double, 2> computeAngleAndDistance(const Point& iPoint, const State& iState);
};
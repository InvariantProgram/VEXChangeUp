#include <math.h>
#include <array>

struct Point {
	double x;
	double y;
};
struct State {
	double x;
	double y;
	double theta;

	bool operator==(const State& rhs) const;
	bool operator!=(const State& rhs) const;
	State operator-(const State& rhs);
	State operator+(const State& rhs);
};

class OdomMath {
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
		* Compute the angle and distance between a point and odometry state
		* All operations in radians, constrained -pi to pi
		* @param iPoint the point
		* @param iState the odom state
		* @return Pair, with (Distance, Angle)
		*/
		static std::array<double, 2> computeAngleAndDistance(const Point& iPoint, const State& iState);
		//Compute and return the difference in x and y, as a pair within an array in the same order
		static std::array<double, 2> computeDiffs(const Point& iPoint, const State& iState);
};
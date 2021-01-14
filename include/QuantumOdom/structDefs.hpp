#pragma once

//ThreeTrackerOdom
struct Chassis {
	//All values in inches
	double WheelDiam;
	double width;
	double midlineOffset;
};
//Xdrive
struct Sensor_vals {
    int right;
    int left;
    int middle;
	bool hasMiddle = false;

	void setVals(int iRight, int iLeft);
    void setVals(int iRight, int iLeft, int iStrafe);
};
//OdomMath
struct Point {
	//Designed to be used in inch, inch
	double x;
	double y;
};
//PIDController
struct PIDConsts {
	double kP;
	double kI;
	double kD;
	double Tf;
};
struct State {
	//Designed to be used in inch, inch, radian
	double x;
	double y;
	double theta;

	bool operator==(const State& rhs) const;
	bool operator!=(const State& rhs) const;
	State operator-(const State& rhs);
	State operator+(const State& rhs);
};
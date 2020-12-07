#pragma once

#include "QuantumOdom/structDefs.hpp"
#include "QuantumOdom/OdomMath.hpp"
#include <array>

class ThreeTrackerOdom {
	private:
		Chassis scales;
		State storedState;
		const double maxDiff = 60;
	public:
		/*
		* Constructs an Odom object
		* Default Constructor: WheelDiam: 4_in, Width: 12.5_in, Offset: 4_in
		* Default Constructor: State with x=0, y=0, theta=0
		*/
		ThreeTrackerOdom();
		/*
		* Constructs an Odom object 
		* Pass in Chassis scales: WheelDiam, Width, and horizontal wheel distance to turning
		*/
		ThreeTrackerOdom(const Chassis& iChassis);
		/*
		* Perform a step for odometry math:
		* @param tickDiffs change in encoder readings: Left, Right, Middle
		* @param dt associated time difference, in milliseconds
		* @return nothing- Updates internal var storedState, retrieved through get function
		*/
		void odomStep(std::array<int, 3> tickDiffs);
		/*
		* Set the stored state to modify based on odom calculations
		* @param iState New state to be set at: x, y, theta
		*/
		void setState(const State& iState);
		/*
		* Set the chassis scales for odom calculations
		* @param iChassis Chassis struct with values WheelDiam, width, and midlineOffset
		*/
		void setChassis(const Chassis& iChassis);
		/*
		* Get the state of the associated odom object:
		* @return State object: ret.x, ret.y, ret.theta
		*/
		State getState();
		/*
		* Get the associated Chassis objec 
		* @return Chassis object: ret.WheelDiam, ret.width, ret.midlineOffset
		*/
		Chassis getChassis();
};
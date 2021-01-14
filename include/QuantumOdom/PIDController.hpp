#pragma once
#include "api.h"
#include "QuantumOdom/structDefs.hpp"

class PIDController {
	private:
		PIDConsts constants;

		double lastOutput;

		double integral;
		double integralLimit;
		
		double lastDiff;
		double target;

		double lastTime;
		double lastDelta;

		double filterWeight;
		double lastDeriv;
		double EMAFilter(double newVal);
	public:
		//Default Constructor
		PIDController();
		//Constructor using only constants
		PIDController(PIDConsts PIDstruct);
		//Full constructor
		PIDController(PIDConsts PIDstruct, double integralLimit, double filterWeight);
		//Change controller constanst
		void setGains(PIDConsts PIDstruct);
		//Windup Guard Change
		void setIntegralLimit(double windupGuard);
		//Set the target and return same val
		double setTarget(double newTarget);
		//Get last dt
		double getMillis();

		//Run one step of the PID control loop
		int step(int inputVal);
};
#include "QuantumOdom/PIDController.hpp"

double PIDController::EMAFilter(double newVal) {
	return (filterWeight * lastDeriv) + (1.0 - filterWeight) * newVal;
}

PIDController::PIDController() {
	integralLimit = 1000;
	filterWeight = 0.5;
}
PIDController::PIDController(PIDConsts PIDstruct) {
	PIDController();
	constants.kP = PIDstruct.kP;
	constants.kI = PIDstruct.kI;
	constants.kD = PIDstruct.kD;
	constants.Tf = PIDstruct.Tf;
	integralLimit = 1000;
	filterWeight = 0.5;
}
PIDController::PIDController(PIDConsts PIDstruct, double integralLimit, double filterWeight) {
	PIDController();
	constants.kP = PIDstruct.kP;
	constants.kI = PIDstruct.kI;
	constants.kD = PIDstruct.kD;
	constants.Tf = PIDstruct.Tf;
	this->integralLimit = integralLimit;
	this->filterWeight = filterWeight;
}

void PIDController::setGains(PIDConsts PIDstruct) {
	constants.kP = PIDstruct.kP;
	constants.kI = PIDstruct.kI;
	constants.kD = PIDstruct.kD;
	constants.Tf = PIDstruct.Tf;
}

void PIDController::setIntegralLimit(double windupGuard) {
	integralLimit = windupGuard;
}

double PIDController::setTarget(double newTarget) {
	target = newTarget;
	lastDeriv = 0;
	lastDiff = 0;
	integral = 0;
	lastOutput = 0;

	lastTime = pros::millis();
	return target;
}

double PIDController::getMillis() {
	return lastDelta;
}

double PIDController::step(double inputVal) {
	double diff = target - inputVal;

	integral += diff;
	if (abs(integral) > integralLimit) integral *= integralLimit / abs(integral);

	double deriv = lastDiff - diff;
	lastDeriv = EMAFilter(deriv);

	lastDelta = (pros::millis() - lastTime) / 1000.0;
	int output = diff * constants.kP + integral * constants.kI * lastDelta + constants.kD * lastDeriv / lastDelta;
	output -= constants.Tf * (output - lastOutput);
	lastOutput = output;
	return output;
}
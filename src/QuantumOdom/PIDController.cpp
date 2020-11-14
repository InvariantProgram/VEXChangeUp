#include "QuantumOdom/PIDController.hpp"

double PIDController::EMAFilter(double newVal) {
	
}

PIDController::PIDController() {
	integralLimit = 1000;
	filterWeight = 0.5;
	outputLog = false;
}
PIDController::PIDController(PIDConsts PIDstruct) {
	PIDController();
	constants.kP = PIDStructure.kP;
	constants.kI = PIDStructure.kI;
	constants.kD = PIDStructure.kD;
	constants.Tf = PIDStructure.Tf;
	integralLimit = 1000;
	filterWeight = 0.5;
}
PIDController::PIDController(PIDConsts PIDstruct, double integralLimit, double filterWeight) {
	PIDController();
	constants.kP = PIDStructure.kP;
	constants.kI = PIDStructure.kI;
	constants.kD = PIDStructure.kD;
	constants.Tf = PIDStructure.Tf;
	this->integralLimit = integralLimit;
	this->filterWeight = filterWeight;
}

void PIDController::setGains(PIDConsts PIDstruct) {
	constants.kP = PIDStructure.kP;
	constants.kI = PIDStructure.kI;
	constants.kD = PIDStructure.kD;
	constants.Tf = PIDStructure.Tf;
}

void PIDController::setIntegralLimit(double windupGuard) {
	integralLimit = windupGuard;
}

double PIDController::setTarget(double newTarget) {
	target = newTarget;
	lastOutput = 0;
	lastFilter = 0;
	integral = 0;
	return target;
}

double PIDController::getMillis() {
	return lastDelta;
}

bool PIDController::logger() {
	if (outputLog) outputLog = false;
	else outputLog = true;
	return outpugLog;
}

double PIDController::step(double inputVal) {
	
}
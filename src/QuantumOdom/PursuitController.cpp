#include "QuantumOdom/PursuitController.hpp"


bool PursuitController::absComp(const double& a, const double& b)
{
	bool FirstLess = false;

	if (abs((int)a) < abs((int)b))
		FirstLess = true;

	return(FirstLess);
}

double PursuitController::angleClamp(double input) {
	double result; double PI = 3.14159;
	int quotient = (int)(input / (2 * PI));
	double remainder = input - quotient * 2 * PI;
	if (remainder < 0) result = 2 * PI - remainder;
	else result = remainder;
	return result;
}

PursuitController::PursuitController(XDrive* iChassis, ThreeTrackerOdom* iOdom,
	PIDController* iForward, PIDController* iTurn) : chassis(iChassis), odomSys(iOdom),
	distCont(iForward), angleCont(iTurn) {}


void PursuitController::toPoint(State newPoint) {
	Point targetLocation = { newPoint.x, newPoint.y };
	distCont->setTarget(0);
	angleCont->setTarget(0);

	double maxMotorVelocity = 200;
	switch (chassis->getGearset()) {
		case pros::E_MOTOR_GEARSET_36:
			maxMotorVelocity = 100;
			break;
		case pros::E_MOTOR_GEARSET_06:
			maxMotorVelocity = 600;
			break;
		default:
			break;
	}

	State currentState;
	std::vector<std::vector<double>> input;

	double translateSpeed, max, maxCoeff, rotateSpeed, theta, forwardCoeff, strafeCoeff;
	std::array<double, 4> output;

	bool running = true;
	while (running) {
		currentState = odomSys->getState();

		//Calculate Forward and Strafe ratios
		theta = currentState.theta;
		std::vector<double> r1 = { newPoint.x - currentState.x };
		std::vector<double> r2 = { newPoint.y - currentState.y };
		input = {r1, r2};
		Matrix diffMat(input);
		r1 = { cos(theta), sin(theta) };
		r2 = { -sin(theta), cos(theta) };
		input = {r1, r2};
		Matrix inverseMat(input);

		Matrix coeffMat = inverseMat * diffMat;

		maxCoeff = coeffMat.getAbsMax();

		forwardCoeff = coeffMat(0, 0) / maxCoeff;
		strafeCoeff = coeffMat(1, 0) / maxCoeff;

		//Get Output Velocities
		translateSpeed = -1 * distCont->step(OdomMath::computeDistance(targetLocation, currentState));
		rotateSpeed = angleCont->step(OdomMath::computeAngle(currentState, newPoint));

		output = { (forwardCoeff + strafeCoeff) * translateSpeed - rotateSpeed, (forwardCoeff - strafeCoeff) * translateSpeed - rotateSpeed,
			(forwardCoeff - strafeCoeff) * translateSpeed + rotateSpeed, (forwardCoeff + strafeCoeff) * translateSpeed + rotateSpeed };

		max = *std::max_element(output.begin(), output.end(), absComp);
		if (abs(max) > maxMotorVelocity) {
			for (int i = 0; i < 4; i++) {
				output[i] *= maxMotorVelocity / abs(max);
			}
		}

		chassis->runMotors(output);

		if (pow(diffMat.getSum(2), 0.5) < errorBounds) {
			chassis->stop(true);
			running = false;
		}

		pros::delay(20);
	}
}

void PursuitController::toPoint(Point newPoint) {
	distCont->setTarget(0);

	double maxMotorVelocity = 200;
	switch (chassis->getGearset()) {
	case pros::E_MOTOR_GEARSET_36:
		maxMotorVelocity = 100;
		break;
	case pros::E_MOTOR_GEARSET_06:
		maxMotorVelocity = 600;
		break;
	default:
		break;
	}

	State currentState;
	std::vector<std::vector<double>> input;

	double translateSpeed, max, theta, maxCoeff, forwardCoeff, strafeCoeff;
	std::array<double, 4> output;

	bool running = true;
	while (running) {
		currentState = odomSys->getState();

		theta = currentState.theta;
		//Calculate Forward and Strafe ratios
		std::vector<double> r1 = { newPoint.x - currentState.x };
		std::vector<double> r2 = { newPoint.y - currentState.y };
		input = { r1, r2 };
		Matrix diffMat(input);
		r1 = { cos(theta), sin(theta) };
		r2 = { -sin(theta), cos(theta) };
		input = { r1, r2 };
		Matrix inverseMat(input);

		Matrix coeffMat = inverseMat * diffMat;

		maxCoeff = coeffMat.getAbsMax();

		forwardCoeff = coeffMat(0, 0) / maxCoeff;
		strafeCoeff = coeffMat(1, 0) / maxCoeff;

		//Get Output Velocities
		translateSpeed = -1 * distCont->step(OdomMath::computeDistance(newPoint, currentState));

		output = { (forwardCoeff + strafeCoeff) * translateSpeed, (forwardCoeff - strafeCoeff) * translateSpeed,
			(forwardCoeff - strafeCoeff) * translateSpeed, (forwardCoeff + strafeCoeff) * translateSpeed };

		max = *std::max_element(output.begin(), output.end(), absComp);
		if (abs(max) > maxMotorVelocity) {
			for (int i = 0; i < 4; i++) {
				output[i] *= maxMotorVelocity / abs(max);
			}
		}

		chassis->runMotors(output);

		if (pow(diffMat.getSum(2), 0.5) < errorBounds) {
			chassis->stop(true);
			running = false;
		}

		pros::delay(20);
	}
}

void PursuitController::toAngle(double newAngle) {
	State newPoint = odomSys->getState();
	newPoint.theta = newAngle;

	angleCont->setTarget(0);

	double maxMotorVelocity = 200;
	switch (chassis->getGearset()) {
	case pros::E_MOTOR_GEARSET_36:
		maxMotorVelocity = 100;
		break;
	case pros::E_MOTOR_GEARSET_06:
		maxMotorVelocity = 600;
		break;
	default:
		break;
	}

	State currentState;
	std::vector<std::vector<double>> input;

	double max, rotateSpeed, theta;
	std::array<double, 4> output;

	bool running = true;
	while (running) {
		currentState = odomSys->getState();

		//Calculate Forward and Strafe ratios

		//Get Output Velocities
		theta = OdomMath::computeAngle(currentState, newPoint);
		rotateSpeed = angleCont->step(theta);

		output = { -rotateSpeed, -rotateSpeed,
			rotateSpeed, rotateSpeed };

		max = *std::max_element(output.begin(), output.end(), absComp);
		if (abs(max) > maxMotorVelocity) {
			for (int i = 0; i < 4; i++) {
				output[i] *= maxMotorVelocity / abs(max);
			}
		}

		chassis->runMotors(output);

		if (abs(theta) * 180 / 3.14159 < errorBounds) {
			chassis->stop(true);
			running = false;
		}

		pros::delay(20);
	}
}

void PursuitController::changeError(double iError) {
	errorBounds = iError;
}

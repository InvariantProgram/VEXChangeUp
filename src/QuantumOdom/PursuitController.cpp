#include "QuantumOdom/PursuitController.hpp"


bool absComp(const double& a, const double& b)
{
	bool FirstLess = false;

	if (abs((int)a) < abs((int)b))
		FirstLess = true;

	return(FirstLess);
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

	double translateSpeed, max, rotateSpeed, theta, forwardCoeff, strafeCoeff;
	std::array<double, 4> output;

	while (true) {
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

		forwardCoeff = (inverseMat(0, 0) > 0) ? 1 : -1;
		strafeCoeff = inverseMat(0, 1) / abs(inverseMat(0, 0));

		//Get Output Velocities
		translateSpeed = distCont->step(OdomMath::computeDistance(targetLocation, currentState));
		rotateSpeed = angleCont->step(OdomMath::computeAngle(currentState, newPoint));

		if (abs(translateSpeed < 5)) {
			chassis->stop();
			return;
		}

		output = { (forwardCoeff + strafeCoeff) * translateSpeed - rotateSpeed, (forwardCoeff - strafeCoeff) * translateSpeed - rotateSpeed,
			(forwardCoeff - strafeCoeff) * translateSpeed + rotateSpeed, (forwardCoeff + strafeCoeff) * translateSpeed + rotateSpeed };

		max = std::max_element(output.begin(), output.end(), absComp);
		if (abs(max) > maxMotorVelocity) {
			for (int i = 0; i < 4; i++) {
				output[i] *= 600 / abs(max);
			}
		}

		chassis->runMotors(output);
		
		pros::delay(10);
	}
}

void PursuitController::changeError(double iError) {
	errorBounds = iError;
}
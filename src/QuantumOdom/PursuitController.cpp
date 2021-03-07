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


void PursuitController::toPoint(State newPoint, double speed, double slowRange=-1) {
	Point targetLocation = { newPoint.x, newPoint.y };

	bool slowEnd = (slowRange > 0);

	bool running = true;
	State currentState;
	std::vector<std::vector<double>> input;

	double ds, dtheta, output;
	double theta, forwardRatio, strafeRatio, total;
	std::array<double, 4> outputVelocities;
	std::array<int, 4> driveVelocities;

	double calcSpeed, turnSpeed, max;

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

	while (running) {
		currentState = odomSys->getState();

		//Calculate Forward and Strafe ratios
		theta = currentState.theta;
		std::vector<double> r1 = { newPoint.x - currentState.x };
		std::vector<double> r2 = { newPoint.y - currentState.y };
		input = {r1, r2};
		Matrix diffMat(input);
		r1 = { -cos(theta), -sin(theta) };
		r2 = { -sin(theta), -cos(theta) };
		input = {r1, r2};
		Matrix inverseMat(input);
		inverseMat *= (1 / pow(cos(theta), 2) - pow(sin(theta), 2));

		Matrix coeffMat = inverseMat * diffMat;

		total = coeffMat.getSum();
		forwardRatio = coeffMat(0, 0) / total;
		strafeRatio = coeffMat(0, 1) / total;


		//Get Output Velocities
		ds = OdomMath::computeDistance(targetLocation, currentState);
		dtheta = OdomMath::computeAngle(currentState, newPoint);

		if (ds < errorBounds) {
			if (slowEnd) chassis->stop(true);
			running = false;
		}

		if (slowEnd && ds < slowRange) calcSpeed = -distCont->step(ds);
		else calcSpeed = speed;

		turnSpeed = angleCont->step(dtheta);

		outputVelocities = {forwardRatio - strafeRatio, forwardRatio + strafeRatio,
			forwardRatio + strafeRatio, forwardRatio - strafeRatio};

		max = abs(*(std::max_element(outputVelocities.begin(), outputVelocities.end(), absComp)));

		for (int i = 0; i < 4; i++) {
			outputVelocities[i] *= calcSpeed / max;
			if (i < 2) outputVelocities[i] -= turnSpeed;
			else outputVelocities[i] += turnSpeed;
		}
		max = abs(*(std::max_element(outputVelocities.begin(), outputVelocities.end(), absComp)));
		if (max > maxMotorVelocity) {
			for (int i = 0; i < 4; i++) {
				outputVelocities[i] *= 600 / max;
			}
		}
		for (int i = 0; i < 4; i++) {
			driveVelocities[i] = (int)outputVelocities[i];
		}

		chassis->runMotors(driveVelocities);
 
		pros::delay(5);
	}
}

void PursuitController::changeError(double iError) {
	errorBounds = iError;
}
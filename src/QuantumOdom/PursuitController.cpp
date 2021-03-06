#include "QuantumOdom/PursuitController.hpp"

PursuitController::PursuitController(XDrive* iChassis, ThreeTrackerOdom* iOdom,
	PIDController* iForward, PIDController* iTurn) : chassis(iChassis), odomSys(iOdom), 
	distCont(iForward), angleCont(iTurn) {}


void toPoint(State newPoint) {
	bool running = true;
	State currentState;
	std::vector<std::vector<double>> input;

	double theta, forwardRatio, strafeRatio, turnRatio;

	Matrix diffMat, inverseMat, coeffMat;

	while (running) {
		currentState = odomSys->getState();

		//Calculate Forward and Strafe ratios
		theta = currentState.theta;
		input = { {newPoint.x - currentState.x}, 
			{newPoint.y - currentState.y} };
		diffMat(input);
		input = { {-cos(theta), -sin(theta)}, {-sin(theta), -cos(theta)} };
		input *= (1 / pow(cos(theta), 2) - pow(sin(theta), 2));
		inverseMat(input);

		coeffMat = inverseMat * diffMat;

		forwardRatio = coeffMat(0, 0);
		strafeRatio = coeffMat(0, 1);


 
		pros::delay(5);
	}
}
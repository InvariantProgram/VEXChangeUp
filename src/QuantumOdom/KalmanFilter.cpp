#include "QuantumOdom/KalmanFilter.hpp"

KalmanFilter::KalmanFilter(Matrix stateTransition, Matrix conversionMatrix, Matrix iObsMat,
	Matrix iProcessNoise, Matrix iMeasurementNoise, Matrix iStateError, int stateLength) {
	F_mat = stateTransition;
	G_mat = conversionMatrix;

	Q_mat = iProcessNoise;
	R_mat = iMeasurementNoise;

	P_k = iStateError;

	H_mat = iObsMat;

	State = Matrix(stateLength, 1);
}

Matrix KalmanFilter::step(std::vector<double> input, std::vector<double> measurement) {
	Matrix in_mat({ input });
	in_mat = in_mat.Transpose();

	//Propagate State to form prediction
	Matrix preState = F_mat * State + G_mat * in_mat;

	//Propagate State Error to get measure of "inaccuracies" in guess
	Matrix P_forward = F_mat * P_k * F_mat.Transpose() + Q_mat;

	//Calculate the Kalman Gain
	Matrix innerResult = H_mat * P_forward * H_mat.Transpose() + R_mat;
	Matrix Gains = P_forward * H_mat.Transpose() * innerResult.inverse();
	
	//"Correct" return state based off of measurement
	Matrix u_mat({ measurement });
	u_mat = u_mat.Transpose();

	Matrix innovation = u_mat + H_mat * preState * -1;
	Matrix output = preState + Gains * innovation;
	this->State = output;

	//Update Process Noise Matrix
	Matrix temp = Matrix::eye(P_forward.get_height()) + Gains * H_mat * -1;
	P_k = temp * P_forward;

	return output;
}

void KalmanFilter::setState(std::vector<double> newState) {
	if (newState.size() == State.get_height()) {
		Matrix temp({ newState });
		State = temp.Transpose();
	}
}

Matrix KalmanFilter::getState() {
	return this->State;
}

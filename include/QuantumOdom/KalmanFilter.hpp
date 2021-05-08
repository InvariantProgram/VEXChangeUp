#pragma once

#include <vector>
#include "Matrix.hpp"

class KalmanFilter {
	private:
		Matrix State;

		Matrix F_mat; //State k-1 to State k
		Matrix G_mat; //Input contribution from k-1 to k

		Matrix Q_mat; //Process noise matrix
		Matrix R_mat; //Measurement noise matrix

		Matrix P_k; //State Error Covariance

		Matrix H_mat; //Observation Matrix (State to output)
	public:
		/*
		* Create a Kalman Filter
		* @param stateTransition: Equations relating state from k-1 to k
		* @param conversionMatrix: Equations about changing state (k-1 to k) from input vars
		* @param iObsMat: Observation Matrix (Convert State to Measurement)
		* @param iProcessNoise: Process Noise Covariance Matrix
		* @param iMeasurementNoise: Measurement Noise Covariance Matrix
		* @param iStateError: Initial Error Covariance Matrix
		* @param stateLength: length of State vector
		*/
		KalmanFilter(Matrix stateTransition, Matrix conversionMatrix, Matrix iObsMat,
			Matrix iProcessNoise, Matrix iMeasurementNoise, Matrix iStateError, int stateLength);

		Matrix step(std::vector<double> input, std::vector<double> measurement);

		void setState(std::vector<double> newState);

		Matrix getState();
};
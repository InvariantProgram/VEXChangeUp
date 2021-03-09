#include "QuantumOdom/Matrix.hpp"

Matrix::Matrix(int M, int N)
{
	height = M;
	width = N;
	mtrx.resize(M);
	for (int i = 0; i < mtrx.size(); i++) {
		mtrx[i].resize(N, 0);
	}
}

Matrix::Matrix(const Matrix& input) {
	mtrx = input.mtrx;
	height = input.get_height();
	width = input.get_width();
}

Matrix::Matrix(std::vector<std::vector<double>> input) {
	mtrx = input;
	height = mtrx.size();
	width = mtrx[0].size();
}

double Matrix::getSum(double power=1) {
	double retVal = 0;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			retVal += pow(mtrx[i][j], power);
		}
	}
	return retVal;
}

double Matrix::getAbsMax() {
	double maxVal = -1;
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			if (abs(mtrx[i][j]) > maxVal) maxVal = abs(mtrx[i][j]);
		}
	}
	return maxVal;
}

double Matrix::operator()(int i, int j) {
	return mtrx[i][j];
}

Matrix Matrix::operator*(const Matrix& rhs) {
	if (this->width != rhs.height) throw "Non-matching Dimensions";
	Matrix result(this->height, rhs.width);

	for (int i = 0; i < result.height; i++) {
		for (int j = 0; j < result.width; j++) {
			double value = 0;
			for (int iter = 0; iter < this->width; iter++) {
				value += mtrx[i][iter] * rhs.mtrx[iter][j];
			}
			result.mtrx[i][j] = value;
		}
	}
	return result;
}
Matrix Matrix::operator+(const Matrix& rhs) {
	if (this->width != rhs.width || this->height != rhs.height)
		throw "Non-matching Dimensions";
	Matrix result(this->height, this->width);
	for (int i = 0; i < result.height; i++) {
		for (int j = 0; j < result.width; j++) {
			result.mtrx[i][j] = mtrx[i][j] + rhs.mtrx[i][j];
		}
	}
	return result;
}
Matrix& Matrix::operator+=(const Matrix& rhs) {
	if (this->width != rhs.width || this->height != rhs.height)
		throw "Non-matching Dimensions";
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			this->mtrx[i][j] += rhs.mtrx[i][j];
		}
	}
	return *this;
}
Matrix Matrix::operator*(const double& rhs) {
	Matrix result(height, width);
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			result.mtrx[i][j] = mtrx[i][j] * rhs;
		}
	}
	return result;
}
Matrix& Matrix::operator*=(const double& rhs) {
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			mtrx[i][j] *= rhs;
		}
	}
	return *this;
}
Matrix Matrix::operator+(const double& rhs) {
	Matrix result(height, width);
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			result.mtrx[i][j] = mtrx[i][j] + rhs;
		}
	}
	return result;
}
Matrix& Matrix::operator+=(const double& rhs) {
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			mtrx[i][j] += rhs;
		}
	}
	return *this;
}

int Matrix::get_height() const {
	return height;
}
int Matrix::get_width() const {
	return width;
}
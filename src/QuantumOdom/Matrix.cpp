#include "QuantumOdom/Matrix.hpp"

Matrix Matrix::eye(int N) {
	Matrix retVal(N, N);
	for (int i = 0; i < N; i++) {
		retVal.changeIndex(i, i, 1);
	}
	return retVal;
}

Matrix::Matrix() {
	height = 1;
	width = 1;
	mtrx.resize(1);
	for (int i = 0; i < mtrx.size(); i++) {
		mtrx[i].resize(1, 0);
	}
}

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

void Matrix::changeIndex(int i, int j, double input) {
	mtrx[i][j] = input;
}

Matrix Matrix::Transpose()
{
	Matrix temp(this->width, this->height);
	for (int i = 0; i < this->height; i++) {
		for (int j = 0; j < this->width; j++) {
			temp.changeIndex(j, i, mtrx[i][j]);
		}
	}
	return temp;
}

Matrix Matrix::getMinor(int row, int col) {
	Matrix retVal(this->width - 1, this->height - 1);
	int columnCount = 0, rowCount = 0;
	for (int i = 0; i < this->height; i++) {
		columnCount = 0;
		if (i == row) continue;
		for (int j = 0; j < this->width; j++) {
			if (j == col) continue;
			retVal.changeIndex(rowCount, columnCount, this->mtrx[i][j]);
			columnCount++;
		}
		rowCount++;
	}
	return retVal;
}

double Matrix::calcDeterminant(Matrix src, int ord) {
	if (ord < 1) throw "Non-Positive Dimension";
	if (ord == 1) return src(0, 0);
	double tot = 0;
	for (int i = 0; i < src.get_width(); i++) {
		tot += pow(-1, i) * src(0, i) * calcDeterminant(src.getMinor(0, i), ord - 1);
	}
	return tot;
}
double Matrix::calcDeterminant() {
	if (this->width != this->height) throw "Not Square Matrix";
	Matrix cpy(*this);
	return calcDeterminant(cpy, this->height);
}

Matrix Matrix::inverse() {
	if (this->width != this->height) throw "Not Square Matrix";
	Matrix retVal(this->height, this->width);
	for (int i = 0; i < retVal.get_height(); i++) {
		for (int j = 0; j < retVal.get_width(); j++) {
			double val = pow(-1, i + j) * this->getMinor(j, i).calcDeterminant();
			retVal.changeIndex(i, j, val);
		}
	}
	if (this->calcDeterminant() == 0) throw "Singular/Degenerate Matrix";
	retVal *= 1 / this->calcDeterminant();
	return retVal;
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
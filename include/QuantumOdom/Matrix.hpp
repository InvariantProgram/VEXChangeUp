#pragma once

#include <vector>

class Matrix {
	private:
		std::vector<std::vector<double>> mtrx;
		int height;
		int width;
	public:
		/*
		* Initialize a matrix of dimensions M x N with default values of 0
		* @param M: Height of Matrix
		* @param N: Width of Matrix
		*/
		Matrix(int M, int N);
		/*
		* Initialize a matrix from a 2D Vector
		* @param input: 2D Vector to be stored
		*/
		Matrix(std::vector<std::vector<double>> input);

		Matrix(const Matrix& input);

		double operator()(int i, int j);

		Matrix operator*(const Matrix& rhs);
		Matrix operator+(const Matrix& rhs);
		Matrix& operator+=(const Matrix& rhs);

		Matrix operator*(const double& rhs);
		Matrix& operator*=(const double& rhs);
		Matrix operator+(const double& rhs);
		Matrix& operator+=(const double& rhs);

		int get_height() const;
		int get_width() const;
};
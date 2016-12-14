
// Leats squares approximate method

#ifndef LEAST_SQUARES
#define LEAST_SQUARES

#include "ApproximateMethod.h"


namespace function {

class LeastSquares : public ApproximateMethod
{
public:

	int approximate();
	double value(double x);

	LeastSquares(int exponent);
	~LeastSquares();

private:

	double fi(double x);
	double q(int j, double x);	

	double getA(int j);
	double getBeta(int j);
	double getAlfa(int j);

	int exponent; // poryadok(stepen') polinoma
	double* a;
	double* alfa;
	double* beta;

	int additionalSummands;

};

}

#endif
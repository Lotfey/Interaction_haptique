#ifndef Solver__h
#define Solver__h

#include <Eigen\Core>

#include "Energy.h"

class Solver
{
public:
	Solver(Energy *energy, const Eigen::VectorXd &mask) { _energy = energy;  _mask = mask; }
	virtual void Minimize(Eigen::VectorXd &x) = 0;
	void TestGradient(const Eigen::VectorXd &x, const double &delta);
	void TestHessian(const Eigen::VectorXd &x, const double &delta, const bool &type = 0);

protected:
	Eigen::VectorXd _mask;
	Energy * _energy;
};

class NewtonMethod : public Solver
{
public:
	NewtonMethod(Energy *energy, const Eigen::VectorXd &mask) : Solver(energy, mask) {}
	void Minimize(Eigen::VectorXd &x);

private:
};

#endif
#ifndef Energy__h
#define Energy__h

#include <Eigen\Core>

class Energy
{
public:
	typedef std::vector<vtkIdType> VertexIndiceVector;
	typedef std::vector<bool> FixedVerticesMaskVector;

	virtual double value(const Eigen::VectorXd &x) const = 0;
	virtual void gradient(const Eigen::VectorXd &x, Eigen::VectorXd &grad) const = 0;
	virtual void hessian(const Eigen::VectorXd &x, Eigen::MatrixXd &hess) const = 0;

	void UpdateY(const Eigen::VectorXd &y) { _y = y; }
	void UpdateExternalForces(Eigen::Vector3d point, vtkIdType id, bool force)
	{
		_movingPoint = point;
		_movingVertexId = id;
		_externalForces = force;
	}

	void Initialize(const double & g, const double &h)
	{
		_g = g;  _h = h;
	}

protected:
	Eigen::VectorXd _y;
	double _g, _h;
	Eigen::Vector3d _movingPoint;
	vtkIdType _movingVertexId;
	bool _externalForces;
};

class FEMEnergy : public Energy
{
public:
	typedef std::vector<Eigen::Matrix3d> MaterialDeformationVector;
	typedef std::vector<double> TriangleAreaVector;
	typedef std::vector<Eigen::SparseMatrix<double>> TensorVector;

	virtual double value(const Eigen::VectorXd &x) const;
	virtual void gradient(const Eigen::VectorXd &x, Eigen::VectorXd &grad) const;
	virtual void hessian(const Eigen::VectorXd &x, Eigen::MatrixXd &hess) const;

	void InitializeFEMEnergy(const VertexIndiceVector& a, const VertexIndiceVector& b, const VertexIndiceVector& c, const MaterialDeformationVector& Dm, const TriangleAreaVector& w, const TensorVector& G, const double& k)
	{
		_a = a; _b = b; _c = c; _Dm = Dm; _w = w; _G = G; _k = k;
	}

private:
	VertexIndiceVector _a, _b, _c;
	MaterialDeformationVector _Dm;
	TriangleAreaVector _w;
	TensorVector _G;
	double _k;
};

#endif
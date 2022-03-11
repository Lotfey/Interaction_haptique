#ifndef ParticleSystem__h
#define ParticleSystem__h

#include <Eigen\Core>

#include "Energy.h"
#include "Solver.h"

class ParticleSystem
{
public:
	typedef std::vector<vtkIdType> VertexIndiceVector;
	typedef std::vector<bool> FixedVerticesMaskVector;

	virtual void MassUpdate() = 0;
	virtual Eigen::VectorXd ComputeVelocity(const Eigen::VectorXd &p) const = 0;

	void Test();
	void Iterate();
	void PolyDataDisplayUpdate() { _polyData->GetPoints()->Modified(); _polyData->Modified(); }
	void GetNormalAtPoint(const vtkIdType &pointId, double normal[3]);
	double * GetMovingVertex()
	{
		if (_movingVertexId == -1) return NULL;
		else return _x.block<3, 1>(3 * _movingVertexId, 0).data();
	}
	void UpdateMovingVertexId(vtkIdType id) { _movingVertexId = id; }
	void UpdateMovingPoint(Eigen::Vector3d point) { _movingPoint = point; }
	void SetExternalForces(bool force) { _externalForces = force; }
	void IntersectWithLine(double origin[3], double direction[3], double point[3], vtkIdType &pointId);

protected:
	ParticleSystem(Energy * energy, Solver * solver, vtkPolyData* polyData, const Eigen::VectorXd &mask, const double& h, const double& m = 1, const double& g = 10);
	~ParticleSystem();

	Energy * _energy;
	Solver * _solver;
	Eigen::SparseMatrix<double> _M;
	vtkIdType FindPointId(vtkPolyData *polyData, double point[3]);
	void KroneckerProductSparse(const Eigen::SparseMatrix<double> &A, const  Eigen::SparseMatrix<double> &B, Eigen::SparseMatrix<double> &AB);
	Eigen::VectorXd _x, _p;
	Eigen::Map<Eigen::VectorXd> * _map;
	vtkPolyData * _polyData;
	double _g, _h, _m;
	vtkIdType _movingVertexId;
	Eigen::Vector3d _movingPoint;
	bool _externalForces;
};

class FiniteElement : public ParticleSystem
{
public:
	typedef std::vector<Eigen::Matrix3d> MaterialDeformationVector;
	typedef std::vector<double> TriangleAreaVector;
	typedef std::vector<Eigen::SparseMatrix<double>> TensorVector;

	FiniteElement(FEMEnergy *energy, Solver * solver, vtkPolyData* polyData, const Eigen::VectorXd &mask, const double& h, const double& m = 1, const double& g = 10, const double& k = 1);
	virtual void MassUpdate();
	virtual Eigen::VectorXd ComputeVelocity(const Eigen::VectorXd &p) const;

private:
	VertexIndiceVector _a, _b, _c;
	MaterialDeformationVector _Dm;
	TriangleAreaVector _w;
	TensorVector _G;
	double _k;
};

#endif
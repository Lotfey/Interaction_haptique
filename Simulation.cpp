#include <vtkSmartPointer.h>
#include <vtkObjectFactory.h>
#include <vtkConeSource.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkPointData.h>
#include <vtkCellLocator.h>
#include <vtkGenericCell.h>
#include <vtkTriangle.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkLine.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkProperty.h>
#include <vtkFloatArray.h>
#include <vtkDoubleArray.h>
#include <vtkExtractEdges.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPoints.h>
#include <vtkTriangleFilter.h>
#include <vtkTransform.h>
#include <vtk3DSImporter.h>
#include <vtkRendererCollection.h>
#include <vtkPointPicker.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#define HAVE_STRUCT_TIMESPEC // To avoid the error of timespec redefinition
#include <HAPI/AnyHapticsDevice.h>
#include <HAPI/HAPISurfaceObject.h>
#include <HAPI/HapticPrimitive.h>
#include <HAPI/FrictionSurface.h>
#include <HAPI/GodObjectRenderer.h>
#include <HAPI/HaptionHapticsDevice.h>
#include <HAPI/HapticForceField.h>

#include "Energy.h"
#include "Solver.h"
#include "ParticleSystem.h"
#include "Interactor.h"

using namespace Eigen;

using namespace HAPI;

vtkStandardNewMacro(MyMouseInteractorStyle);

MyMouseInteractorStyle::MyMouseInteractorStyle()
{
	this->LeftButtonDown = false;
	this->MotionEnabled = false;
	this->HapticUse = false;
}

void MyMouseInteractorStyle::OnTimer()
{
	// Haptic device interaction
	if (HapticUse)
	{
		// Retrieve haptic device position and orientation
		Vec3 pos = HapticsDevice->getPosition();
		Rotation rotation = HapticsDevice->getOrientation();
		Vec3 axis = rotation.axis;
		double angle = 180. * rotation.angle / std::acos(-1.0);
		
		// Apply the transform to the cone avatar
		vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
		transform->Translate(pos.x, pos.y, pos.z);
		transform->RotateWXYZ(angle, axis.x, axis.y, axis.z);
		ConeActor->SetUserTransform(transform);

		// Mise à jour de la position du bras haptique
		movingPoint = Eigen::Vector3d(pos.x, pos.y, pos.z);

		// 3D picking with the axis of the cone
		if (!MotionEnabled)
		{
			Vec3 dir = rotation * Vec3(0.0, 0.0, 1.0);
			double direction[3] = { dir.x, dir.y, dir.z };
			double position[3] = { pos.x, pos.y, pos.z };
			double intersection[3];
			PS->IntersectWithLine(position, direction, intersection, pointId);
			if (pointId != -1) PickedActor->SetPosition(intersection);
			PS->UpdateMovingVertexId(pointId);
		}
	}

	// Mouse interaction
	else
	{
/*
		// 2D picking
		vtkPointPicker * pointPicker = vtkPointPicker::SafeDownCast(this->GetInteractor()->GetPicker());
		int x = this->GetInteractor()->GetEventPosition()[0];
		int y = this->GetInteractor()->GetEventPosition()[1];
		vtkRenderer *aren = this->GetInteractor()->FindPokedRenderer(x, y);
		pointPicker->Pick(x, y, 0, aren);
		double *point = pointPicker->GetPickPosition();
		this->pointId = pointPicker->GetPointId();
		if (this->pointId == 0) this->pointId = -1;
		cout << "Point id: " << this->pointId << endl;
		this->movingPoint = Eigen::Vector3d(point[0], point[1], point[2]);

		// Transform of the avatar
		// Align it with the surface normal in case of mouse interaction
		// SetOrientation requires three angles for the rotations around the axes Z, X and Y applied in that order
		ConeActor->SetPosition(point);
		double direction[3];
		this->GetPS()->GetNormalAtPoint(this->pointId, direction);
		double angleZ = atan2(direction[1], direction[0]) * 180.0 / vtkMath::Pi();
		double angleY = atan2(sqrt(direction[0] * direction[0] + direction[1] * direction[1]), direction[2]) * 180.0 / vtkMath::Pi();
		ConeActor->SetOrientation( 0, angleY, angleZ ); 

		// Picker actor
		if (this->pointId!=-1) 
		{
			PickedActor->VisibilityOn();
			PickedActor->SetPosition(point);
		}
		else
		{
			if (!MotionEnabled) PickedActor->VisibilityOff();
		}
*/
	}

	// SetOrientation requires three angles for the rotations around the axes Z, X and Y applied in that order

	// Particle system update
	if (MotionEnabled)
	{
		this->GetPS()->MassUpdate();
		this->GetPS()->Iterate();
		this->GetPS()->PolyDataDisplayUpdate();

		// Update the position of the picked actor
		double* movingVertex = PS->GetMovingVertex();
		if (movingVertex) PickedActor->SetPosition(movingVertex);

		if (LeftButtonDown)
		{
			// Update moving point position
			PS->UpdateMovingPoint(movingPoint);

			// Update spring position
			if (spring && movingVertex)
			{
				Vec3 position(movingVertex[0], movingVertex[1], movingVertex[2]);
				spring->setPosition(position);
			}
		}
	}

	GetInteractor()->Render();
}

void MyMouseInteractorStyle::OnLeftButtonDown()
{
	cout << "Down" << endl;

	LeftButtonDown = true;

	this->GetPS()->SetExternalForces(true);

	if (!MotionEnabled) vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

void MyMouseInteractorStyle::OnLeftButtonUp()
{
	cout << "Up" << endl;

	LeftButtonDown = false;

	this->GetPS()->SetExternalForces(false);

	vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}

void MyMouseInteractorStyle::OnKeyPress()
{
	if (strcmp(GetInteractor()->GetKeySym(), "m") == 0)
	{
		MotionEnabled = !MotionEnabled;

		if (MotionEnabled)
		{
			// Add a spring haptic force
			if (HapticUse)
			{
				spring = new HapticSpring(Vec3(movingPoint.x(), movingPoint.y(), movingPoint.z()), 20.0);
				HapticsDevice->addEffect(spring);
				HapticsDevice->transferObjects();
			}
		}
		else
		{
			// Remove spring haptic force
			if (HapticUse)
			{
				HapticsDevice->removeEffect(spring);
				HapticsDevice->transferObjects();
			}
		}
	}
}

double Energy::value(const VectorXd &x) const
{
	double E = 0.0;

	// Gravity potential
	//for (int k = 0; k < x.size(); k+=3)
	//{
	//	E -= _g * x[k + 2];
	//}

	// Spring potential energy
	if (_externalForces && _movingVertexId != -1)
	{
		E += 0.5 * 1000 * (x.block<3, 1>(3 * _movingVertexId, 0) - _movingPoint).squaredNorm();
	}

	return 0.5 * (x - _y).squaredNorm() + _h * _h * E;
}

void Energy::gradient(const VectorXd& x, VectorXd& grad) const
{
	grad = x - _y;

	// Gravity potential
	//for (int k = 0; k < x.size(); k += 3)
	//{
	//	grad[k + 2] -= _h * _h * _g;
	//}

	// Spring force
	if (_externalForces && _movingVertexId != -1)
	{
		grad.block<3, 1>(3 * _movingVertexId, 0) +=
			_h * _h * 1000 * (x.block<3, 1>(3 * _movingVertexId, 0) - _movingPoint);
	}
}

void Energy::hessian(const VectorXd &x, MatrixXd &hess) const
{
	hess = MatrixXd::Identity(x.size(), x.size());

	// Spring force derivative
	if (_externalForces && _movingVertexId != -1)
	{
		Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
		hess.block<3, 3>(3 * _movingVertexId, 3 * _movingVertexId) += 1000 * _h * _h * I3;
	}
}

double FEMEnergy::value(const VectorXd &x) const
{
	double E0 = Energy::value(x), E = 0.0;

	Matrix3d I3 = Matrix3d::Identity();
	Map<VectorXd> vI3(I3.data(), I3.size());

	// Elastic energy
	VertexIndiceVector::const_iterator ita, itb, itc;
	TriangleAreaVector::const_iterator itw;
	TensorVector::const_iterator itG;
	for (ita = _a.begin(), itb = _b.begin(), itc = _c.begin(), itw = _w.begin(), itG = _G.begin(); ita < _a.end(); ++ita, ++itb, ++itc, ++itw, ++itG)
	{
		E += _k * *itw * (*itG * x - vI3).squaredNorm();
	}

	return E0 + _h * _h * E;
}

void FEMEnergy::gradient(const VectorXd &x, VectorXd &grad) const
{
	Energy::gradient(x, grad);

	Matrix3d I3 = Matrix3d::Identity();
	Map<VectorXd> vI3(I3.data(), I3.size());

	// Elastic energy
	VertexIndiceVector::const_iterator ita, itb, itc;
	TriangleAreaVector::const_iterator itw;
	TensorVector::const_iterator itG;
	for (ita = _a.begin(), itb = _b.begin(), itc = _c.begin(), itG = _G.begin(), itw = _w.begin(); ita < _a.end(); ++ita, ++itb, ++itc, ++itG, ++itw)
	{
		grad += 2.0 * _h * _h * _k * *itw * itG->transpose() * (*itG * x - vI3);
	}
}

void FEMEnergy::hessian(const VectorXd &x, MatrixXd &hess) const
{
	Energy::hessian(x, hess);

	// Elastic energy
	VertexIndiceVector::const_iterator ita, itb, itc;
	TriangleAreaVector::const_iterator itw;
	TensorVector::const_iterator itG;
	for (ita = _a.begin(), itb = _b.begin(), itc = _c.begin(), itw = _w.begin(), itG = _G.begin(); ita < _a.end(); ++ita, ++itb, ++itc, ++itw, ++itG)
	{
		hess += 2.0 * _h * _h * _k * *itw * itG->transpose() * *itG;
	}
}

void Solver::TestGradient(const VectorXd &x0, const double &delta)
{
	double f0 = _energy->value(x0);
	VectorXd df;
	df.resize(x0.size());
	_energy->gradient(x0, df);
	for (int i = 0; i < x0.size(); i++)
	{
		VectorXd dx(x0);
		dx(i) += delta;
		double fd = (_energy->value(dx) - f0) / delta;
		cout << i << " th " << df(i) << " fd " << fd << endl;
		getchar();
	}
}

void Solver::TestHessian(const VectorXd &x0, const double &delta, const bool &type)
{
	// Finite difference from function value
	if (type == 0)
	{
		double f0 = _energy->value(x0);
		MatrixXd d2f;
		d2f.resize(x0.size(), x0.size());
		_energy->hessian(x0, d2f);
		for (int i = 0; i < x0.size(); i++)
		{
			VectorXd dxi(x0);
			dxi(i) += delta;
			double fi = _energy->value(dxi);
			for (int j = 0; j < x0.size(); j++)
			{
				VectorXd dxj(x0);
				dxj(j) += delta;
				double fj = _energy->value(dxj);
				VectorXd dxij(x0);
				dxij(i) += delta;
				dxij(j) += delta;
				double fij = _energy->value(dxij);
				double fd = (fij - fi - fj + f0) / delta / delta;
				cout << i << " " << j << " th " << d2f(i, j) << " fd " << fd << endl;
				getchar();
			}
		}
	}
	// Finite difference from gradient	
	else
	{
		VectorXd grad0(x0.size());
		_energy->gradient(x0, grad0);
		MatrixXd d2f;
		d2f.resize(x0.size(), x0.size());
		_energy->hessian(x0, d2f);
		for (int i = 0; i < x0.size(); i++)
		{
			VectorXd dxi(x0);
			dxi(i) += delta;
			VectorXd gradi(x0.size());
			_energy->gradient(dxi, gradi);
			for (int j = 0; j < x0.size(); j++)
			{
				VectorXd dxj(x0);
				dxj(j) += delta;
				VectorXd gradj(x0.size());
				_energy->gradient(dxj, gradj);
				double fd = 0.5 * (gradi(j) - grad0(j)) / delta + 0.5 * (gradj(i) - grad0(i)) / delta;
				cout << i << " " << j << " th " << d2f(i, j) << " fd " << fd << endl;
				getchar();
			}
		}
	}
}

void NewtonMethod::Minimize(VectorXd &x)
{
	VectorXd Gradient(x.size());
	_energy->gradient(x, Gradient);

	MatrixXd Hessian(x.size(), x.size());
	_energy->hessian(x, Hessian);

	LDLT<MatrixXd> ldlt(Hessian);
	if (ldlt.isNegative()) cout << "Negative definite Hessian" << endl;
	x -= _mask.cwiseProduct(ldlt.solve(Gradient));
}

ParticleSystem::ParticleSystem(Energy * energy, Solver * solver, vtkPolyData* polyData, const VectorXd& mask, const double& h, const double& m, const double& g)
	: _energy(energy), _solver(solver), _polyData(polyData), _h(h), _m(m), _g(g)
{
	// Initialization of the position vector x from the initial polydata geometry
	vtkSmartPointer<vtkDoubleArray> array = vtkDoubleArray::SafeDownCast(_polyData->GetPoints()->GetData());
	double *x0 = static_cast<double *>(array->GetVoidPointer(0));
	_map = new Map<VectorXd>(x0, 3 * _polyData->GetNumberOfPoints());
	_x = *_map;

	// Initialization of the linear momentum vector v
	_p = VectorXd(3 * _polyData->GetNumberOfPoints());
	_p.setZero();

	// Allocation of the mass matrix
	_M = SparseMatrix<double>(_x.size(), _x.size());

	_energy->Initialize(_g, _h);
}

ParticleSystem::~ParticleSystem()
{
	delete _map;
}

vtkIdType ParticleSystem::FindPointId(vtkPolyData *polyData, double point[3])
{
	for (int k = 0; k < polyData->GetNumberOfPoints(); k++)
	{
		double p[3];
		polyData->GetPoint(k, p);
		if (fabs(p[0] - point[0])<1.e-6 && fabs(p[1] - point[1])<1.e-6 && fabs(p[2] - point[2])<1.e-6) return k;
	}
	return -1;
}

void ParticleSystem::KroneckerProductSparse(const SparseMatrix<double> &A, const  SparseMatrix<double> &B, SparseMatrix<double> &AB)
{
	const unsigned int Ar = A.rows(),
		Ac = A.cols(),
		Br = B.rows(),
		Bc = B.cols();
	AB.resize(Ar*Br, Ac*Bc);
	AB.resizeNonZeros(0);
	AB.reserve(A.nonZeros()*B.nonZeros());
	for (int kA = 0; kA<A.outerSize(); ++kA)
	{
		for (int kB = 0; kB<B.outerSize(); ++kB)
		{
			for (SparseMatrix<double>::InnerIterator itA(A, kA); itA; ++itA)
			{
				for (SparseMatrix<double>::InnerIterator itB(B, kB); itB; ++itB)
				{
					const unsigned int iA = itA.row(),
						jA = itA.col(),
						iB = itB.row(),
						jB = itB.col(),
						i = iA*Br + iB,
						j = jA*Bc + jB;
					AB.insert(i, j) = itA.value() * itB.value();
				}
			}
		}
	}
}

void ParticleSystem::Test()
{
	VectorXd x0 = _x + 0.1 * VectorXd::Random(_x.size());
	cout << x0 << endl;
	VectorXd y = VectorXd::Zero(_x.size());
	_energy->UpdateY(y);
	_solver->TestGradient(x0, 1.e-6);
	_solver->TestHessian(x0, 1.e-6);
}

void ParticleSystem::Iterate()
{
	VectorXd x0(_x), p0(_p);
	VectorXd v0 = ComputeVelocity(p0);
	VectorXd y = x0 + _h * v0;
	_energy->UpdateY(y);
	_energy->UpdateExternalForces(this->_movingPoint, this->_movingVertexId, this->_externalForces);
	_solver->Minimize(_x);
	
	VectorXd v = (_x - x0) / _h;
 	_p = _M * v;

	// Polydata point coordinates update
	*_map = _x;
}

void ParticleSystem::GetNormalAtPoint(const vtkIdType &pointId, double normal[3])
{
	vtkSmartPointer<vtkFloatArray> pointNormalsRetrieved = vtkFloatArray::SafeDownCast(_polyData->GetPointData()->GetNormals());
	if (pointNormalsRetrieved)
	{ 
		pointNormalsRetrieved->GetTuple(pointId, normal);
	}
	else 
	{
		normal[0] = 0;
		normal[1] = 0;
		normal[2] = 1;
	}
}

void ParticleSystem::IntersectWithLine(double origin[3], double direction[3], double point[3], vtkIdType &pointId)
{
	vtkSmartPointer<vtkCellLocator> cellLocator = vtkSmartPointer<vtkCellLocator>::New();
	cellLocator->SetDataSet(_polyData);
	cellLocator->BuildLocator();
	double final[3] = { origin[0] + direction[0], origin[1] + direction[1], origin[2] + direction[2] };
	double t = 0.0, pline[3] = {0.0, 0.0, 0.0}, pcoords[3] = {0.0, 0.0, 0.0};
	vtkGenericCell *cell = vtkGenericCell::New();
	int subId = 0;
	vtkIdType cellId = -1;
	if (cellLocator->IntersectWithLine(origin, final, 0.001, t, pline, pcoords, subId, cellId, cell) != 0)
	{
		//cout << "Cell Id: " << cellId << endl;
		vtkSmartPointer<vtkIdList> cellPointsId = vtkSmartPointer<vtkIdList>::New();
		_polyData->GetCellPoints(cellId, cellPointsId);
		vtkIdType i0 = cellPointsId->GetId(0);
		double *p0 = _polyData->GetPoint(i0);
		double d2 = (pcoords[0] - p0[0]) * (pcoords[0] - p0[0]) + (pcoords[1] - p0[1]) * (pcoords[1] - p0[1]) + (pcoords[2] - p0[2]) * (pcoords[2] - p0[2]);
		pointId = i0; point[0] = p0[0]; point[1] = p0[1]; point[2] = p0[2]; 
		vtkIdType i1 = cellPointsId->GetId(1);
		double *p1 = _polyData->GetPoint(i1);
		double d12 = (pcoords[0] - p1[0]) * (pcoords[0] - p1[0]) + (pcoords[1] - p1[1]) * (pcoords[1] - p1[1]) + (pcoords[2] - p1[2]) * (pcoords[2] - p1[2]);
		if (d12 < d2) { d2 = d12; pointId = i1; point[0] = p1[0]; point[1] = p1[1]; point[2] = p1[2]; }
		vtkIdType i2 = cellPointsId->GetId(2);
		double *p2 = _polyData->GetPoint(i2);
		double d22 = (pcoords[0] - p2[0]) * (pcoords[0] - p2[0]) + (pcoords[1] - p2[1]) * (pcoords[1] - p2[1]) + (pcoords[2] - p2[2]) * (pcoords[2] - p2[2]);
		if (d22 < d2) { d2 = d22; pointId = i2; point[0] = p2[0]; point[1] = p2[1]; point[2] = p2[2]; }
		//cout << "Point Id: " << pointId << endl;
	}
	// If no intersection is found, set the output point to the origin
	else 
	{	
		point[0] = origin[0];
		point[1] = origin[1];
		point[2] = origin[2];
		pointId = -1;
	}
}

FiniteElement::FiniteElement(FEMEnergy *energy, Solver * solver, vtkPolyData* polyData, const VectorXd& mask, const double& h, const double& m, const double& g, const double& k)
	: ParticleSystem(energy, solver, polyData, mask, h, m, g), _k(k)
{
	vtkCellArray *cellArray = polyData->GetPolys();
	vtkIdType numCells = cellArray->GetNumberOfCells();
	vtkIdType cellLocation = 0; // the index into the cell array
	for (vtkIdType i = 0; i < numCells; i++)
	{
		vtkIdType numIds; // to hold the size of the cell
		const vtkIdType *pointIds; // to hold the ids in the cell
		cellArray->GetCell(cellLocation, numIds, pointIds);
		cellLocation += 1 + numIds;

		_a.push_back(pointIds[0]);
		_b.push_back(pointIds[1]);
		_c.push_back(pointIds[2]);
	}

	SparseMatrix<double> sI3 = Matrix3d::Identity().sparseView();
	VertexIndiceVector::const_iterator ita, itb, itc;
	for (ita = _a.begin(), itb = _b.begin(), itc = _c.begin(); ita < _a.end(); ita++, itb++, itc++)
	{
		// Rest length of the spring corresponding to the edge
		Vector3d xaxb = _x.block<3, 1>(*itb * 3, 0) - _x.block<3, 1>(*ita * 3, 0);
		Vector3d xaxc = _x.block<3, 1>(*itc * 3, 0) - _x.block<3, 1>(*ita * 3, 0);
		Vector3d n = xaxb.cross(xaxc);
		Matrix3d Dm;
		Dm.block<1, 3>(0, 0) = xaxb;
		Dm.block<1, 3>(1, 0) = xaxc;
		Dm.block<1, 3>(2, 0) = n; // Unit normal as third vector
		
		// Initialisation of the initial deformation matrix
		Matrix3d Dmi = Dm.inverse();
		_Dm.push_back(Dmi);

		// Initialisation of the triangle area equal to half of the cross-product norm
		_w.push_back(0.5 * n.norm()); // 

		// Initialisation of the tensor matrix
		SparseMatrix<double> sDmi = Dmi.sparseView();
		typedef Triplet<double> T;
		SparseMatrix<double> S(3, _x.size() / 3);
		std::vector<T> coef;
		coef.push_back(T(0, *ita, -1));
		coef.push_back(T(1, *ita, -1));
		coef.push_back(T(0, *itb, 1));
		coef.push_back(T(1, *itc, 1));
		S.setFromTriplets(coef.begin(), coef.end());

		SparseMatrix<double> G;
		KroneckerProductSparse(sDmi * S, sI3, G);

		_G.push_back(G);
	}

	energy->InitializeFEMEnergy(_a, _b, _c, _Dm, _w, _G, _k);
}

void FiniteElement::MassUpdate()
{
	_M = SparseMatrix<double>(_x.size(), _x.size());
	_M.reserve(_x.size() * _x.size());

	typedef Triplet<double> T;
	std::vector<T> coef;

	VertexIndiceVector::const_iterator ita, itb, itc;
	TriangleAreaVector::const_iterator itw;
	for (ita = _a.begin(), itb = _b.begin(), itc = _c.begin(), itw = _w.begin(); ita < _a.end(); ita++, itb++, itc++, itw++)
	{
		// Rest length of the spring corresponding to the edge
		Vector3d xaxb = _x.block<3, 1>(*itb * 3, 0) - _x.block<3, 1>(*ita * 3, 0);
		Vector3d xaxc = _x.block<3, 1>(*itc * 3, 0) - _x.block<3, 1>(*ita * 3, 0);
		double A = 0.5 * xaxb.cross(xaxc).norm();

		for (int k = 0; k < 3; k++)
		{
			_M.coeffRef(*ita * 3 + k, *ita * 3 + k) += A / 6;
			_M.coeffRef(*ita * 3 + k, *itb * 3 + k) += A / 12;
			_M.coeffRef(*ita * 3 + k, *itc * 3 + k) += A / 12;
			_M.coeffRef(*itb * 3 + k, *ita * 3 + k) += A / 12;
			_M.coeffRef(*itb * 3 + k, *itb * 3 + k) += A / 6;
			_M.coeffRef(*itb * 3 + k, *itc * 3 + k) += A / 12;
			_M.coeffRef(*itc * 3 + k, *ita * 3 + k) += A / 12;
			_M.coeffRef(*itc * 3 + k, *itb * 3 + k) += A / 12;
			_M.coeffRef(*itc * 3 + k, *itc * 3 + k) += A / 6;
		}
	}

	_M.makeCompressed();
}

VectorXd FiniteElement::ComputeVelocity(const VectorXd& p) const
{
	SimplicialLLT<SparseMatrix<double>> llt(_M);
	VectorXd v = llt.solve(p);
	double norm = (p - _M * v).norm();

	return v;
}

int main(int, char* [])
{
	// If the haptic device is detected, use it
	// otherwise use the mouse instead to pull in the normal direction
	bool hapticUse = true;

	// Create a new haptics device, using any device connected.
	std::unique_ptr<HaptionHapticsDevice> device(new HaptionHapticsDevice("127.0.0.1"));

	// The haptics renderer to use.
	device->setHapticsRenderer( new GodObjectRenderer() );
	
	// initialize the device
	if( device->initDevice() != HAPIHapticsDevice::SUCCESS ) 
	{
		// initilization failed, print error message
		cerr << device->getLastErrorMsg() << endl;
		hapticUse = false;
	}

	// x -> [-0.25,0.25] y -> [-0.15,0.25] z -> [0.15,0.35]
	HAPI::Matrix4 m = device->getPositionCalibration();
	m.setElement(2, 3, m.getElement(2, 3) - 0.25);
	device->setPositionCalibration(m);
	
	// enable the device (forces and positions will be updated)
	if (hapticUse) 
	{	
		device->enableDevice();
		
		// Add a constant force field
		//device->addEffect(new HapticForceField(Vec3(1.0, 0.0, 0.0)));
		//device->transferObjects();
	}

	// Create an avatar as a cone
	vtkSmartPointer<vtkConeSource> coneSource = vtkSmartPointer<vtkConeSource>::New();
	coneSource->SetHeight(0.1);
	coneSource->SetRadius(0.01);
	coneSource->SetResolution(20);
	coneSource->Update();

	vtkSmartPointer<vtkTransform> coneTransform = vtkSmartPointer<vtkTransform>::New();
	coneTransform->RotateY(90); // x-axis becomes z-axis
	coneTransform->Translate(-coneSource->GetHeight() / 2, 0.0, 0.0);

	vtkSmartPointer<vtkTransformPolyDataFilter> filter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
	filter->SetTransform(coneTransform);
	filter->SetInputConnection(coneSource->GetOutputPort());
	filter->Update();

	vtkSmartPointer<vtkPolyDataMapper> coneMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	coneMapper->SetInputConnection(filter->GetOutputPort());
	vtkSmartPointer<vtkActor> coneActor = vtkSmartPointer<vtkActor>::New();
	coneActor->GetProperty()->SetColor(0.0, 0.0, 1.0);
	coneActor->SetMapper(coneMapper);

	// Create an avatar for the picked point
	vtkSmartPointer<vtkSphereSource> pickedSource = vtkSmartPointer<vtkSphereSource>::New();
	pickedSource->SetRadius(0.01);
	pickedSource->SetPhiResolution(10);
	pickedSource->SetThetaResolution(10);
	pickedSource->Update();

	vtkSmartPointer<vtkPolyDataMapper> pickedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	pickedMapper->SetInputConnection(pickedSource->GetOutputPort());
	vtkSmartPointer<vtkActor> pickedActor = vtkSmartPointer<vtkActor>::New();
	pickedActor->GetProperty()->SetColor(0.0, 0.0, 1.0);
	pickedActor->SetMapper(pickedMapper);

	// Test with sphere source
	vtkSmartPointer<vtkSphereSource> sphereSource = vtkSmartPointer<vtkSphereSource>::New();
	sphereSource->SetRadius(0.1);
	sphereSource->SetOutputPointsPrecision(vtkAlgorithm::DOUBLE_PRECISION);
	sphereSource->Update();

	vtkSmartPointer<vtkPolyData> spherePolydata = sphereSource->GetOutput();

	// Create a mapper and actor for the sphere
	vtkSmartPointer<vtkPolyDataMapper> sphereMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	sphereMapper->SetInputData(spherePolydata);

	vtkSmartPointer<vtkActor> sphereActor =	vtkSmartPointer<vtkActor>::New();
	sphereActor->SetMapper(sphereMapper);

	// Fixed vertices
	int nFixed;
	cout << "Number of fixed vertices [1.." << spherePolydata->GetNumberOfPoints() << "]: ";
	cin >> nFixed;
	cout << "Enter the " << nFixed << " vertices [0.." << spherePolydata->GetNumberOfPoints() - 1 << "]:" << endl;
	std::vector<vtkIdType> fixed; fixed.resize(nFixed);
	for (int k = 0; k < nFixed; k++)
	{
		cout << "\tFixed vertex #" << k << ": ";
		cin >> fixed[k];
	}

	// Convert fixed vertices to mask
	VectorXd mask = VectorXd::Ones(3 * spherePolydata->GetNumberOfPoints());
	for (unsigned int k = 0; k < fixed.size(); k++)
	{
		vtkIdType ind = fixed[k];
		mask[3 * ind] = 0.0;
		mask[3 * ind + 1] = 0.0;
		mask[3 * ind + 2] = 0.0;
	}

	// Particle system instance
	Energy *energy = new FEMEnergy();
	Solver *solver = new NewtonMethod(energy, mask);
	ParticleSystem *ps = new FiniteElement((FEMEnergy*)energy, solver, spherePolydata, mask, 0.01, 0.1, 10.0, 500.0);

	// Create a mapper and actor
	vtkSmartPointer<vtkPolyDataMapper> sphereMapperSurf =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	sphereMapperSurf->SetInputData(spherePolydata);
	sphereMapperSurf->SetResolveCoincidentTopologyPolygonOffsetParameters(0, 1);
	sphereMapperSurf->SetResolveCoincidentTopologyToPolygonOffset();
	vtkSmartPointer<vtkActor> sphereActorSurf = vtkSmartPointer<vtkActor>::New();
	sphereActorSurf->SetMapper(sphereMapperSurf);
	sphereActorSurf->GetProperty()->SetColor(1.0, 0.0, 0.0);
	sphereActorSurf->GetProperty()->SetRepresentationToSurface();

	vtkSmartPointer<vtkPolyDataMapper> sphereMapperEdge =
		vtkSmartPointer<vtkPolyDataMapper>::New();
	sphereMapperEdge->SetInputData(spherePolydata);
	sphereMapperEdge->SetResolveCoincidentTopologyPolygonOffsetParameters(1, 1);
	sphereMapperEdge->SetResolveCoincidentTopologyToPolygonOffset();
	sphereMapperEdge->ScalarVisibilityOff();
	vtkSmartPointer<vtkActor> sphereActorEdge = vtkSmartPointer<vtkActor>::New();
	sphereActorEdge->SetMapper(sphereMapperEdge);
	sphereActorEdge->GetProperty()->SetColor(1.0, 0.0, 0.0);
	sphereActorEdge->GetProperty()->SetRepresentationToWireframe();
	sphereActorEdge->GetProperty()->SetAmbient(1.0);
	sphereActorEdge->GetProperty()->SetDiffuse(0.0);
	sphereActorEdge->GetProperty()->SetSpecular(0.0);

	// Create small spheres at fixed points
	std::vector<vtkSmartPointer<vtkActor> > actors;
	vtkPoints *points = spherePolydata->GetPoints();

	for (unsigned int i = 0; i < fixed.size(); i++)
	{
		vtkSmartPointer<vtkSphereSource> sphereSource =
			vtkSmartPointer<vtkSphereSource>::New();
		double *p = points->GetPoint(fixed[i]);
		sphereSource->SetCenter(p[0], p[1], p[2]);
		sphereSource->SetRadius(.01);

		vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(sphereSource->GetOutputPort());

		vtkSmartPointer<vtkActor> actor =
			vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(mapper);
		actor->GetProperty()->SetColor(0.0, 1.0, 0.0);

		actors.push_back(actor);
	}

	// Create a renderer, render window, and interactor
	vtkSmartPointer<vtkRenderer> renderer =
	vtkSmartPointer<vtkRenderer>::New();
	vtkSmartPointer<vtkRenderWindow> renderWindow =
	vtkSmartPointer<vtkRenderWindow>::New();
	renderWindow->AddRenderer(renderer);
	vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
	vtkSmartPointer<vtkRenderWindowInteractor>::New();
	renderWindowInteractor->SetRenderWindow(renderWindow);
	
	// Stereo
	renderWindow->StereoCapableWindowOn();
	renderWindow->SetStereoTypeToCrystalEyes();
	renderWindow->StereoRenderOn();

	// Add the actor to the scene
	renderer->AddActor(sphereActorSurf);
	renderer->AddActor(sphereActorEdge);
	for (unsigned int i = 0; i < fixed.size(); i++)	renderer->AddActor(actors[i]);
	renderer->SetBackground(1,1,1); // Background color white
	
	// Add the actors of the avatars
	renderer->AddActor(coneActor);
	renderer->AddActor(pickedActor);

	// Display coordinate axes
	vtkSmartPointer<vtkAxesActor> axes =
		vtkSmartPointer<vtkAxesActor>::New();

	vtkSmartPointer<vtkOrientationMarkerWidget> widget =
		vtkSmartPointer<vtkOrientationMarkerWidget>::New();
	widget->SetOutlineColor(0.9300, 0.5700, 0.1300);
	widget->SetOrientationMarker(axes);
	widget->SetInteractor(renderWindowInteractor);
	//widget->SetViewport(0.0, 0.0, 0.4, 0.4);
	widget->SetEnabled(1);
	widget->InteractiveOn();

	// Render and interact
	renderWindow->Render();

	// Initialize must be called prior to creating timer events.
	renderWindowInteractor->Initialize();

	vtkSmartPointer<MyMouseInteractorStyle> myMouseInteractorStyle = MyMouseInteractorStyle::New();
	myMouseInteractorStyle->SetPS(ps);
	myMouseInteractorStyle->SetHapticUse(hapticUse);
	if (hapticUse) myMouseInteractorStyle->SetHapticsDevice(device.get());
	
	// Pass the avatars actor to the interactor style class
	myMouseInteractorStyle->SetConeActor(coneActor);
	myMouseInteractorStyle->SetPickedActor(pickedActor);
	
	renderWindowInteractor->SetInteractorStyle(myMouseInteractorStyle);
	vtkSmartPointer<vtkPointPicker> pointPicker = vtkSmartPointer<vtkPointPicker>::New();
	renderWindowInteractor->SetPicker(pointPicker);

	int timerId = renderWindowInteractor->CreateRepeatingTimer(30);

	// Start the interaction and timer
	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

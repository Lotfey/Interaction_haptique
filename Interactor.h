#ifndef Interactor__h
#define Interactor__h

#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkActor.h>

#include <HAPI/HAPIHapticsDevice.h>
#include <HAPI/HapticTriangleSet.h>
#include <HAPI/HapticSpring.h>

#include "ParticleSystem.h"

class HapticVTKPolyData;

class MyMouseInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
	static MyMouseInteractorStyle* New();
	vtkTypeMacro(MyMouseInteractorStyle, vtkInteractorStyleTrackballCamera);

	MyMouseInteractorStyle();

	virtual void OnTimer();
	
	virtual void OnLeftButtonDown();

	virtual void OnLeftButtonUp();

	virtual void OnKeyPress();

	vtkGetMacro(PS, ParticleSystem *);
	vtkSetMacro(PS, ParticleSystem *);

	vtkGetMacro(HapticsDevice, HAPI::HAPIHapticsDevice *);
	vtkSetMacro(HapticsDevice, HAPI::HAPIHapticsDevice *);

	vtkGetMacro(HapticUse, bool);
	vtkSetMacro(HapticUse, bool);

	vtkGetMacro(ConeActor, vtkActor*);
	vtkSetMacro(ConeActor, vtkActor*);

	vtkGetMacro(PickedActor, vtkActor*);
	vtkSetMacro(PickedActor, vtkActor*);

protected:
	ParticleSystem * PS;
	bool LeftButtonDown, HapticUse, MotionEnabled;
	HAPI::HAPIHapticsDevice *HapticsDevice; 
	HAPI::HapticSpring *spring;
	vtkIdType pointId;
	Eigen::Vector3d movingPoint;
	vtkActor* ConeActor, * PickedActor;
};

#endif
///////////////////////////////////////////////////////////////////////////////
// SixAxisArticulatedTrafo.cpp
#include "TcPch.h"
#pragma hdrstop

#include "SixAxisArticulatedTrafo.h"
#include "TC_DriverVersion.h"

// https://github.com/AndrewPst/6DOF_inverse_kinematics
// https://github.com/ModySaggaf/Forward-Kinematics-for-6-DoF-Robotic-Arm
#include "Matrix.h"
#include "Kinematics.h"

#include <cmath>
#include <vector>
#include <array>


///////////////////////////////////////////////////////////////////////////////
// Collection of interfaces implemented by module CSixAxisArticulatedTrafo
BEGIN_INTERFACE_MAP(CSixAxisArticulatedTrafo)
	INTERFACE_ENTRY_ITCOMOBJECT()
	INTERFACE_ENTRY(IID_ITcADI, ITcADI)
	INTERFACE_ENTRY(IID_ITcWatchSource, ITcWatchSource)
///<AutoGeneratedContent id="InterfaceMap">
	INTERFACE_ENTRY(IID_ITcNcTrafo, ITcNcTrafo)
///</AutoGeneratedContent>
END_INTERFACE_MAP()

IMPLEMENT_IPERSIST_LIB(CSixAxisArticulatedTrafo, VID_TC_Driver, CID_TC_DriverCSixAxisArticulatedTrafo)
IMPLEMENT_ITCOMOBJECT(CSixAxisArticulatedTrafo)
IMPLEMENT_ITCOMOBJECT_SETSTATE_LOCKOP2(CSixAxisArticulatedTrafo)
IMPLEMENT_ITCADI(CSixAxisArticulatedTrafo)
IMPLEMENT_ITCWATCHSOURCE(CSixAxisArticulatedTrafo)

///////////////////////////////////////////////////////////////////////////////
// Set parameters of CSixAxisArticulatedTrafo 
BEGIN_SETOBJPARA_MAP(CSixAxisArticulatedTrafo)
	SETOBJPARA_DATAAREA_MAP()
///<AutoGeneratedContent id="SetObjectParameterMap">
	SETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmLengthL1, m_ArmLengthL1)
	SETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmLengthL2, m_ArmLengthL2)
	SETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmLengthL3, m_ArmLengthL3)
	SETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD1, m_ArmOffsetD1)
	SETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD2, m_ArmOffsetD2)
	SETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD3, m_ArmOffsetD3)
///</AutoGeneratedContent>
END_SETOBJPARA_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get parameters of CSixAxisArticulatedTrafo 
BEGIN_GETOBJPARA_MAP(CSixAxisArticulatedTrafo)
	GETOBJPARA_DATAAREA_MAP()
///<AutoGeneratedContent id="GetObjectParameterMap">
	GETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmLengthL1, m_ArmLengthL1)
	GETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmLengthL2, m_ArmLengthL2)
	GETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmLengthL3, m_ArmLengthL3)
	GETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD1, m_ArmOffsetD1)
	GETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD2, m_ArmOffsetD2)
	GETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD3, m_ArmOffsetD3)
///</AutoGeneratedContent>
END_GETOBJPARA_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get watch entries of CSixAxisArticulatedTrafo
BEGIN_OBJPARAWATCH_MAP(CSixAxisArticulatedTrafo)
	OBJPARAWATCH_DATAAREA_MAP()
///<AutoGeneratedContent id="ObjectParameterWatchMap">
///</AutoGeneratedContent>
END_OBJPARAWATCH_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get data area members of CSixAxisArticulatedTrafo
BEGIN_OBJDATAAREA_MAP(CSixAxisArticulatedTrafo)
///<AutoGeneratedContent id="ObjectDataAreaMap">
///</AutoGeneratedContent>
END_OBJDATAAREA_MAP()


///////////////////////////////////////////////////////////////////////////////
// Constructor
CSixAxisArticulatedTrafo::CSixAxisArticulatedTrafo() //: m_forwardNbrIn(6), m_forwardNbrOut(6)
{
///<AutoGeneratedContent id="MemberInitialization">
	m_ArmLengthL1 = 0;
	m_ArmLengthL2 = 0;
	m_ArmLengthL3 = 0;
	m_ArmOffsetD1 = 0;
	m_ArmOffsetD2 = 0;
	m_ArmOffsetD3 = 0;
///</AutoGeneratedContent>
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
CSixAxisArticulatedTrafo::~CSixAxisArticulatedTrafo() 
{
}

///////////////////////////////////////////////////////////////////////////////
// State Transitions 
///////////////////////////////////////////////////////////////////////////////
IMPLEMENT_ITCOMOBJECT_SETOBJSTATE_IP_PI(CSixAxisArticulatedTrafo)

///////////////////////////////////////////////////////////////////////////////
// State transition from PREOP to SAFEOP
//
// Initialize input parameters 
// Allocate memory
HRESULT CSixAxisArticulatedTrafo::SetObjStatePS(PTComInitDataHdr pInitData)
{
	HRESULT hr = S_OK;
	IMPLEMENT_ITCOMOBJECT_EVALUATE_INITDATA(pInitData);
	return hr;
}

///////////////////////////////////////////////////////////////////////////////
// State transition from SAFEOP to OP
//
// Register with other TwinCAT objects
HRESULT CSixAxisArticulatedTrafo::SetObjStateSO()
{
	HRESULT hr = S_OK;
	return hr;
}

///////////////////////////////////////////////////////////////////////////////
// State transition from OP to SAFEOP
HRESULT CSixAxisArticulatedTrafo::SetObjStateOS()
{
	HRESULT hr = S_OK;
	return hr;
}

///////////////////////////////////////////////////////////////////////////////
// State transition from SAFEOP to PREOP
HRESULT CSixAxisArticulatedTrafo::SetObjStateSP()
{
	HRESULT hr = S_OK;
	return hr;
}

///<AutoGeneratedContent id="ImplementationOf_ITcNcTrafo">
HRESULT CSixAxisArticulatedTrafo::Forward(TcNcTrafoParameter* p)
{
	double ACS[6];
	double MCS[6];

	std::memcpy(ACS, p->i, 6 * sizeof(double));

	switch (p->type) {
	case EcNcTrafoParameter_Invalid:
		break;
	case EcNcTrafoParameter_Base:
		break;
	case EcNcTrafoParameter_Ext:
		break;
	case EcNcTrafoParameter_ExtCnc:
		break;
	}

	// Example DH parameters for a 6-axis robot
	/*
		double m_ArmLengthL1;
		double m_ArmLengthL2;
		double m_ArmLengthL3;
		double m_ArmOffsetD1;
		double m_ArmOffsetD2;
		double m_ArmOffsetD3;
	*/

	/*
	* Annin Robotics AR4
	* theta1=0		d1=169.77	a1=64.2		alpha1=90
	* theta1=90		d2=0		a2=305		alpha2=0
	* theta1=0		d3=0		a3=0		alpha3=90
	* theta1=0		d4=222.63	a4=0		alpha4=90
	* theta1=0		d5=0		a5=0		alpha5=-90
	* theta1=0		d6=36.25	a6=0		alpha6=0
	* 
	DH Parameters 
		( \theta ): Rotation angle around the z-axis (in radians).
		( d ): Offset along the previous z-axis (in millimeters).
		( a ): Length of the common normal (in millimeters).
		( \alpha ): Angle around the common normal (in radians).

		{"theta1", "d1", "a1", "alpha1"},
		{"theta2", "d2", "a2", "alpha2"},
		{"theta3", "d3", "a3", "alpha3"},
		{"theta4", "d4", "a4", "alpha4"},
		{"theta5", "d5", "a5", "alpha5"},
		{"theta6", "d6", "a6", "alpha6"}
	*/

	// Setup manipulator parameters
	Manipulator_t<6> man;
	man.alfa = { M_PI_2, 0, M_PI_2, M_PI_2, -M_PI_2, 0 };
	man.theta = { 0, DEG_TO_RAD(90), 0, 0, 0, 0 };
	man.r = { 64.2, 305, 0, 0, 0, 0 };
	man.d = { 169.77, 0, 0, 222.63, 0, 36.25 };

	// init kinematics calculator
	KinematicsCalc kin(std::move(man));
	Position_t pos;

	//  input: t - joints value for the calculation of the forward kinematics
	//  output: out - pos value for the calculation of the forward kinematics
	//kin.forwardKinematicsOptimized({ 0, M_PI_2, -M_PI_2, 0, 0, 0 }, pos);
	kin.forwardKinematicsOptimized({	DEG_TO_RAD(ACS[0]), DEG_TO_RAD(ACS[1]), DEG_TO_RAD(ACS[2]),
										DEG_TO_RAD(ACS[3]), DEG_TO_RAD(ACS[4]), DEG_TO_RAD(ACS[5]) },
										pos);

	std::memcpy(p->o, MCS, 6 * sizeof(double));

	HRESULT hr = S_OK;
	return hr;
}

HRESULT CSixAxisArticulatedTrafo::Backward(TcNcTrafoParameter* p)
{
	double MCS[6];
	double ACS[6];

	std::memcpy(MCS, p->i, 6 * sizeof(double));

	// Example DH parameters for a 6-axis robot
	/*
		double m_ArmLengthL1;
		double m_ArmLengthL2;
		double m_ArmLengthL3;
		double m_ArmOffsetD1;
		double m_ArmOffsetD2;
		double m_ArmOffsetD3;
	*/
	/*
		* Annin Robotics AR4
		* theta1=0		d1=169.77	a1=64.2		alpha1=90
		* theta1=90		d2=0		a2=305		alpha2=0
		* theta1=0		d3=0		a3=0		alpha3=90
		* theta1=0		d4=222.63	a4=0		alpha4=90
		* theta1=0		d5=0		a5=0		alpha5=-90
		* theta1=0		d6=36.25	a6=0		alpha6=0
		*

	*/

	// Setup manipulator parameters
	Manipulator_t<6> man;
	man.alfa = { M_PI_2, 0, M_PI_2, M_PI_2, -M_PI_2, 0 };
	man.theta = { 0, DEG_TO_RAD(90), 0, 0, 0, 0 };
	man.r = { 64.2, 305, 0, 0, 0, 0 };
	man.d = { 169.77, 0, 0, 222.63, 0, 36.25 };

	// init kinematics calculator
	KinematicsCalc kin(std::move(man));
	Position_t pos;

	std::vector<double> out(6);
	kin.inverseKinematicsOptimized(pos, out);

	for (int i = 0; i <= 5; i++) {
		ACS[i] = RAD_TO_DEG(out[i]);
	}

	std::memcpy(p->o, ACS, 6 * sizeof(double));

	HRESULT hr = S_OK;
	return hr;
}

HRESULT CSixAxisArticulatedTrafo::TrafoSupported(TcNcTrafoParameter* p, bool fwd)
{
	HRESULT hr = S_OK;
	return hr;
}

HRESULT CSixAxisArticulatedTrafo::GetDimensions(ULONG* pFwdInput, ULONG* pFwdOutput)
{
	*pFwdInput = 6;
	*pFwdOutput = 6;

	HRESULT hr = S_OK;
	return hr;
}
///</AutoGeneratedContent>

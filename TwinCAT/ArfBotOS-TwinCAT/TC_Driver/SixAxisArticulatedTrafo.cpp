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
	SETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmLengthA1, m_ArmLengthA1)
	SETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmLengthA2, m_ArmLengthA2)
	SETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmLengthA3, m_ArmLengthA3)
	SETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD1, m_ArmOffsetD1)
	SETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD3, m_ArmOffsetD3)
	SETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD4, m_ArmOffsetD4)
	SETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD6, m_ArmOffsetD6)
///</AutoGeneratedContent>
END_SETOBJPARA_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get parameters of CSixAxisArticulatedTrafo 
BEGIN_GETOBJPARA_MAP(CSixAxisArticulatedTrafo)
	GETOBJPARA_DATAAREA_MAP()
///<AutoGeneratedContent id="GetObjectParameterMap">
	GETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmLengthA1, m_ArmLengthA1)
	GETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmLengthA2, m_ArmLengthA2)
	GETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmLengthA3, m_ArmLengthA3)
	GETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD1, m_ArmOffsetD1)
	GETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD3, m_ArmOffsetD3)
	GETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD4, m_ArmOffsetD4)
	GETOBJPARA_VALUE(PID_SixAxisArticulatedTrafoArmOffsetD6, m_ArmOffsetD6)
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
	m_ArmLengthA1 = 0;
	m_ArmLengthA2 = 0;
	m_ArmLengthA3 = 0;
	m_ArmOffsetD1 = 0;
	m_ArmOffsetD3 = 0;
	m_ArmOffsetD4 = 0;
	m_ArmOffsetD6 = 0;
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

	// Setup manipulator parameters
	Manipulator_t<6> man;
	man.alfa = { -M_PI_2, 0, M_PI_2, -M_PI_2, M_PI_2, 0 };
	man.theta = { 0, -M_PI_2, M_PI, -M_PI, 0, M_PI };
	man.r = { m_ArmLengthA1, m_ArmLengthA2, m_ArmLengthA3, 0, 0, 0 };
	man.d = { m_ArmOffsetD1, 0, m_ArmOffsetD3, m_ArmOffsetD4, 0, m_ArmOffsetD6 };

	// init kinematics calculator
	KinematicsCalc kin(std::move(man));
	Position_t pos;

	//  input: t - joints value for the calculation of the forward kinematics
	//  output: out - pos value for the calculation of the forward kinematics
	double ACS[6];
	std::memcpy(ACS, p->i, 6 * sizeof(double));
	kin.forwardKinematicsOptimized({	DEG_TO_RAD(ACS[0]), DEG_TO_RAD(ACS[1]), DEG_TO_RAD(ACS[2]),
										DEG_TO_RAD(ACS[3]), DEG_TO_RAD(ACS[4]), DEG_TO_RAD(ACS[5]) },
										pos);

	std::memcpy(p->o, &pos, 6 * sizeof(double));

	HRESULT hr = S_OK;
	return hr;
}

HRESULT CSixAxisArticulatedTrafo::Backward(TcNcTrafoParameter* p)
{
	// Setup manipulator parameters
	Manipulator_t<6> man;
	man.alfa = { -M_PI_2, 0, M_PI_2, -M_PI_2, M_PI_2, 0 };
	man.theta = { 0, -M_PI_2, M_PI, -M_PI, 0, M_PI };
	man.r = { m_ArmLengthA1, m_ArmLengthA2, m_ArmLengthA3, 0, 0, 0 };
	man.d = { m_ArmOffsetD1, 0, m_ArmOffsetD3, m_ArmOffsetD4, 0, m_ArmOffsetD6 };

	// init kinematics calculator
	KinematicsCalc kin(std::move(man));
	Position_t pos;

	std::vector<double> out(6);
	std::memcpy(&pos, p->i, 6 * sizeof(double));
	kin.inverseKinematicsOptimized(pos, out);

	double ACS[6];
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

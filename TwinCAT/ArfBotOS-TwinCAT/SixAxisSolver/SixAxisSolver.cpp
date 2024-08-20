///////////////////////////////////////////////////////////////////////////////
// SixAxisSolver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "SixAxisSolver.h"
#include "SixAxisSolverVersion.h"

#include "Kinematics.h"
#include "MatrixUtils.h"

#define NumberOfAxes 6

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)
#define RAD_TO_DEG(radians) ((radians) * (180.0 / M_PI))

#define M_PI       3.14159265358979323846   // pi
#define M_PI_2     1.57079632679489661923   // pi/2

static double ACS_Old[NumberOfAxes];

struct Pose {
	double position[3];
	double orientation[3]; // Roll, Pitch, Yaw
};

Pose extractPose(const double transform[4][4]) {
	Pose pose;

	// Extract position
	pose.position[0] = transform[0][3];
	pose.position[1] = transform[1][3];
	pose.position[2] = transform[2][3];

	// Extract orientation (roll, pitch, yaw)
	double sy = sqrt(transform[0][0] * transform[0][0] + transform[1][0] * transform[1][0]);

	bool singular = sy < 1e-6; // If sy is close to zero, singularity is detected

	if (!singular) {
		pose.orientation[0] = atan2(transform[2][1], transform[2][2]); // Roll
		pose.orientation[1] = atan2(-transform[2][0], sy);              // Pitch
		pose.orientation[2] = atan2(transform[1][0], transform[0][0]);  // Yaw
	}
	else {
		pose.orientation[0] = atan2(-transform[1][2], transform[1][1]); // Roll
		pose.orientation[1] = atan2(-transform[2][0], sy);              // Pitch
		pose.orientation[2] = 0;                                        // Yaw
	}

	return pose;
}

void createTransformMatrix(const Pose& pose, double transform[4][4]) {
	double roll = pose.orientation[0];
	double pitch = pose.orientation[1];
	double yaw = pose.orientation[2];

	// Calculate rotation matrix components
	double cr = cos(roll);
	double sr = sin(roll);
	double cp = cos(pitch);
	double sp = sin(pitch);
	double cy = cos(yaw);
	double sy = sin(yaw);

	transform[0][0] = cy * cp;
	transform[0][1] = cy * sp * sr - sy * cr;
	transform[0][2] = cy * sp * cr + sy * sr;
	transform[0][3] = pose.position[0];

	transform[1][0] = sy * cp;
	transform[1][1] = sy * sp * sr + cy * cr;
	transform[1][2] = sy * sp * cr - cy * sr;
	transform[1][3] = pose.position[1];

	transform[2][0] = -sp;
	transform[2][1] = cp * sr;
	transform[2][2] = cp * cr;
	transform[2][3] = pose.position[2];

	transform[3][0] = 0;
	transform[3][1] = 0;
	transform[3][2] = 0;
	transform[3][3] = 1;
}


///////////////////////////////////////////////////////////////////////////////
// Collection of interfaces implemented by module CSixAxisSolver
BEGIN_INTERFACE_MAP(CSixAxisSolver)
	INTERFACE_ENTRY_ITCOMOBJECT()
	INTERFACE_ENTRY(IID_ITcADI, ITcADI)
	INTERFACE_ENTRY(IID_ITcWatchSource, ITcWatchSource)
///<AutoGeneratedContent id="InterfaceMap">
	INTERFACE_ENTRY(IID_ITcNcTrafo, ITcNcTrafo)
///</AutoGeneratedContent>
END_INTERFACE_MAP()

IMPLEMENT_IPERSIST_LIB(CSixAxisSolver, VID_SixAxisSolver, CID_SixAxisSolverCSixAxisSolver)
IMPLEMENT_ITCOMOBJECT(CSixAxisSolver)
IMPLEMENT_ITCOMOBJECT_SETSTATE_LOCKOP2(CSixAxisSolver)
IMPLEMENT_ITCADI(CSixAxisSolver)
IMPLEMENT_ITCWATCHSOURCE(CSixAxisSolver)

///////////////////////////////////////////////////////////////////////////////
// Set parameters of CSixAxisSolver 
BEGIN_SETOBJPARA_MAP(CSixAxisSolver)
	SETOBJPARA_DATAAREA_MAP()
///<AutoGeneratedContent id="SetObjectParameterMap">
	SETOBJPARA_VALUE(PID_SixAxisSolverAxis1Screw, m_Axis1Screw)
	SETOBJPARA_VALUE(PID_SixAxisSolverAxis2Screw, m_Axis2Screw)
	SETOBJPARA_VALUE(PID_SixAxisSolverAxis3Screw, m_Axis3Screw)
	SETOBJPARA_VALUE(PID_SixAxisSolverAxis4Screw, m_Axis4Screw)
	SETOBJPARA_VALUE(PID_SixAxisSolverAxis5Screw, m_Axis5Screw)
	SETOBJPARA_VALUE(PID_SixAxisSolverAxis6Screw, m_Axis6Screw)
	SETOBJPARA_VALUE(PID_SixAxisSolverEndEffectorHome, m_EndEffectorHome)
///</AutoGeneratedContent>
END_SETOBJPARA_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get parameters of CSixAxisSolver 
BEGIN_GETOBJPARA_MAP(CSixAxisSolver)
	GETOBJPARA_DATAAREA_MAP()
///<AutoGeneratedContent id="GetObjectParameterMap">
	GETOBJPARA_VALUE(PID_SixAxisSolverAxis1Screw, m_Axis1Screw)
	GETOBJPARA_VALUE(PID_SixAxisSolverAxis2Screw, m_Axis2Screw)
	GETOBJPARA_VALUE(PID_SixAxisSolverAxis3Screw, m_Axis3Screw)
	GETOBJPARA_VALUE(PID_SixAxisSolverAxis4Screw, m_Axis4Screw)
	GETOBJPARA_VALUE(PID_SixAxisSolverAxis5Screw, m_Axis5Screw)
	GETOBJPARA_VALUE(PID_SixAxisSolverAxis6Screw, m_Axis6Screw)
	GETOBJPARA_VALUE(PID_SixAxisSolverEndEffectorHome, m_EndEffectorHome)
///</AutoGeneratedContent>
END_GETOBJPARA_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get watch entries of CSixAxisSolver
BEGIN_OBJPARAWATCH_MAP(CSixAxisSolver)
	OBJPARAWATCH_DATAAREA_MAP()
///<AutoGeneratedContent id="ObjectParameterWatchMap">
///</AutoGeneratedContent>
END_OBJPARAWATCH_MAP()

///////////////////////////////////////////////////////////////////////////////
// Get data area members of CSixAxisSolver
BEGIN_OBJDATAAREA_MAP(CSixAxisSolver)
///<AutoGeneratedContent id="ObjectDataAreaMap">
///</AutoGeneratedContent>
END_OBJDATAAREA_MAP()


///////////////////////////////////////////////////////////////////////////////
// Constructor
CSixAxisSolver::CSixAxisSolver()
{
///<AutoGeneratedContent id="MemberInitialization">
	memset(&m_Axis1Screw, 0, sizeof(m_Axis1Screw));
	memset(&m_Axis2Screw, 0, sizeof(m_Axis2Screw));
	memset(&m_Axis3Screw, 0, sizeof(m_Axis3Screw));
	memset(&m_Axis4Screw, 0, sizeof(m_Axis4Screw));
	memset(&m_Axis5Screw, 0, sizeof(m_Axis5Screw));
	memset(&m_Axis6Screw, 0, sizeof(m_Axis6Screw));
	memset(&m_EndEffectorHome, 0, sizeof(m_EndEffectorHome));
///</AutoGeneratedContent>
}

///////////////////////////////////////////////////////////////////////////////
// Destructor
CSixAxisSolver::~CSixAxisSolver() 
{
}

///////////////////////////////////////////////////////////////////////////////
// State Transitions 
///////////////////////////////////////////////////////////////////////////////
IMPLEMENT_ITCOMOBJECT_SETOBJSTATE_IP_PI(CSixAxisSolver)

///////////////////////////////////////////////////////////////////////////////
// State transition from PREOP to SAFEOP
//
// Initialize input parameters 
// Allocate memory
HRESULT CSixAxisSolver::SetObjStatePS(PTComInitDataHdr pInitData)
{
	HRESULT hr = S_OK;
	IMPLEMENT_ITCOMOBJECT_EVALUATE_INITDATA(pInitData);
	return hr;
}

///////////////////////////////////////////////////////////////////////////////
// State transition from SAFEOP to OP
//
// Register with other TwinCAT objects
HRESULT CSixAxisSolver::SetObjStateSO()
{
	HRESULT hr = S_OK;
	return hr;
}

///////////////////////////////////////////////////////////////////////////////
// State transition from OP to SAFEOP
HRESULT CSixAxisSolver::SetObjStateOS()
{
	HRESULT hr = S_OK;
	return hr;
}

///////////////////////////////////////////////////////////////////////////////
// State transition from SAFEOP to PREOP
HRESULT CSixAxisSolver::SetObjStateSP()
{
	HRESULT hr = S_OK;
	return hr;
}

///<AutoGeneratedContent id="ImplementationOf_ITcNcTrafo">
HRESULT CSixAxisSolver::Forward(TcNcTrafoParameter* p)
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

	HRESULT hr = TrafoSupported(p, true);
	if (SUCCEEDED(hr))
	{
		if (p->i && p->o)
		{

			Kinematics kin(NumberOfAxes);

			kin.add_joint_axis(m_Axis1Screw.XAlignment, m_Axis1Screw.YAlignment, m_Axis1Screw.ZAlignment, 
				m_Axis1Screw.XDisplacement, m_Axis1Screw.YDisplacement, m_Axis1Screw.ZDisplacement); // axis 1
			kin.add_joint_axis(m_Axis2Screw.XAlignment, m_Axis2Screw.YAlignment, m_Axis2Screw.ZAlignment,
				m_Axis2Screw.XDisplacement, m_Axis2Screw.YDisplacement, m_Axis2Screw.ZDisplacement); // axis 2
			kin.add_joint_axis(m_Axis3Screw.XAlignment, m_Axis3Screw.YAlignment, m_Axis3Screw.ZAlignment,
				m_Axis3Screw.XDisplacement, m_Axis3Screw.YDisplacement, m_Axis3Screw.ZDisplacement); // axis 3
			kin.add_joint_axis(m_Axis4Screw.XAlignment, m_Axis4Screw.YAlignment, m_Axis4Screw.ZAlignment,
				m_Axis4Screw.XDisplacement, m_Axis4Screw.YDisplacement, m_Axis4Screw.ZDisplacement); // axis 4
			kin.add_joint_axis(m_Axis5Screw.XAlignment, m_Axis5Screw.YAlignment, m_Axis5Screw.ZAlignment,
				m_Axis5Screw.XDisplacement, m_Axis5Screw.YDisplacement, m_Axis5Screw.ZDisplacement); // axis 5
			kin.add_joint_axis(m_Axis6Screw.XAlignment, m_Axis6Screw.YAlignment, m_Axis6Screw.ZAlignment,
				m_Axis6Screw.XDisplacement, m_Axis6Screw.YDisplacement, m_Axis6Screw.ZDisplacement); // axis 6

			kin.add_initial_end_effector_pose(
				m_EndEffectorHome.XXAlignment, m_EndEffectorHome.YXAlignment, m_EndEffectorHome.ZXAlignment, m_EndEffectorHome.XDisplacement,
				m_EndEffectorHome.XYAlignment, m_EndEffectorHome.YYAlignment, m_EndEffectorHome.ZYAlignment, m_EndEffectorHome.YDisplacement,
				m_EndEffectorHome.XZAlignment, m_EndEffectorHome.YZAlignment, m_EndEffectorHome.ZZAlignment, m_EndEffectorHome.ZDisplacement,
				0, 0, 0, 1);

			//
			double ACS[NumberOfAxes];
			std::memcpy(ACS, p->i, NumberOfAxes * sizeof(double));

			// convert
			for (int _i = 0; _i <= NumberOfAxes-1; _i++) {
				ACS[_i] = DEG_TO_RAD(ACS[_i]);
			}
			std::memcpy(ACS_Old, ACS, NumberOfAxes * sizeof(double));

			double transform[4][4];
			kin.forward(ACS, (double*)transform);
			Pose pose = extractPose(transform);

			// convert
			double MCS[NumberOfAxes];
			for (int _i = 0; _i <= 2; _i++) {
				MCS[_i] = pose.position[_i];
			}
			for (int _i = 3; _i <= NumberOfAxes-1; _i++) {
				MCS[_i] = RAD_TO_DEG(pose.orientation[_i - 3]);
			}

			std::memcpy(p->o, &MCS, NumberOfAxes * sizeof(double));
		}

		if (p->d_i && p->d_o)
		{
			//
		}

		if (p->dd_i && p->dd_o)
		{
			//
		}
	}
	return hr;
}

HRESULT CSixAxisSolver::Backward(TcNcTrafoParameter* p)
{
	HRESULT hr = TrafoSupported(p, false);
	if (SUCCEEDED(hr))
	{
		if (p->i && p->o)
		{

			Kinematics kin(NumberOfAxes);

			kin.add_joint_axis(m_Axis1Screw.XAlignment, m_Axis1Screw.YAlignment, m_Axis1Screw.ZAlignment,
				m_Axis1Screw.XDisplacement, m_Axis1Screw.YDisplacement, m_Axis1Screw.ZDisplacement); // axis 1
			kin.add_joint_axis(m_Axis2Screw.XAlignment, m_Axis2Screw.YAlignment, m_Axis2Screw.ZAlignment,
				m_Axis2Screw.XDisplacement, m_Axis2Screw.YDisplacement, m_Axis2Screw.ZDisplacement); // axis 2
			kin.add_joint_axis(m_Axis3Screw.XAlignment, m_Axis3Screw.YAlignment, m_Axis3Screw.ZAlignment,
				m_Axis3Screw.XDisplacement, m_Axis3Screw.YDisplacement, m_Axis3Screw.ZDisplacement); // axis 3
			kin.add_joint_axis(m_Axis4Screw.XAlignment, m_Axis4Screw.YAlignment, m_Axis4Screw.ZAlignment,
				m_Axis4Screw.XDisplacement, m_Axis4Screw.YDisplacement, m_Axis4Screw.ZDisplacement); // axis 4
			kin.add_joint_axis(m_Axis5Screw.XAlignment, m_Axis5Screw.YAlignment, m_Axis5Screw.ZAlignment,
				m_Axis5Screw.XDisplacement, m_Axis5Screw.YDisplacement, m_Axis5Screw.ZDisplacement); // axis 5
			kin.add_joint_axis(m_Axis6Screw.XAlignment, m_Axis6Screw.YAlignment, m_Axis6Screw.ZAlignment,
				m_Axis6Screw.XDisplacement, m_Axis6Screw.YDisplacement, m_Axis6Screw.ZDisplacement); // axis 6

			kin.add_initial_end_effector_pose(
				m_EndEffectorHome.XXAlignment, m_EndEffectorHome.YXAlignment, m_EndEffectorHome.ZXAlignment, m_EndEffectorHome.XDisplacement,
				m_EndEffectorHome.XYAlignment, m_EndEffectorHome.YYAlignment, m_EndEffectorHome.ZYAlignment, m_EndEffectorHome.YDisplacement,
				m_EndEffectorHome.XZAlignment, m_EndEffectorHome.YZAlignment, m_EndEffectorHome.ZZAlignment, m_EndEffectorHome.ZDisplacement,
				0, 0, 0, 1);

			Pose pose;
			double transform[4][4];
			double MCS[NumberOfAxes];
			std::memcpy(&MCS, p->i, NumberOfAxes * sizeof(double));

			// convert
			for (int _i = 0; _i <= 2; _i++) {
				pose.position[_i] = MCS[_i];
			}
			for (int _i = 3; _i <= NumberOfAxes-1; _i++) {
				pose.orientation[_i - 3] = DEG_TO_RAD(MCS[_i]);
			}
			createTransformMatrix(pose, transform);

			double jac[6][NumberOfAxes];
			double jac_t[6][NumberOfAxes];
			double AA_t[6][6];
			double A_tA[NumberOfAxes][NumberOfAxes];
			double pinv[NumberOfAxes][6];

			double ACS[NumberOfAxes];
			kin.inverse((double*)transform, (double*)jac, (double*)pinv, (double*)jac_t,
				(double*)AA_t, (double*)A_tA, ACS_Old, 0.01, 0.001, 20,
				ACS);
			std::memcpy(ACS_Old, ACS, NumberOfAxes * sizeof(double));

			// convert
			for (int i = 0; i <= NumberOfAxes-1; i++) {
				ACS[i] = RAD_TO_DEG(ACS[i]);
			}
			std::memcpy(p->o, ACS, NumberOfAxes * sizeof(double));

		}

		if (p->d_i && p->d_o)
		{
			//
		}

		if (p->dd_i && p->dd_o)
		{
			//
		}
	}
	return hr;
}

HRESULT CSixAxisSolver::TrafoSupported(TcNcTrafoParameter* p, bool fwd)
{
	HRESULT hr = S_OK;
	/*
	* NCERR_KINTRAFO_OFFSET
	* NCERR_KINTRAFO_FAILED
	* NCERR_KINTRAFO_AMBIGUOUSSOLUTION
	* NCERR_KINTRAFO_INVALIDAXISPOS
	* NCERR_KINTRAFO_INVALIDDIM
	*/

	if (p)
	{
		if (fwd)
		{
			if (p->dim_i != NumberOfAxes || p->dim_o != NumberOfAxes)
			{
				hr = MAKE_ADS_HRESULT(NCERR_KINTRAFO_INVALIDDIM); // kinematics transformation error: invalid dimension
			}
		}
		else
		{
			if (p->dim_i != NumberOfAxes || p->dim_o != NumberOfAxes)
			{
				hr = MAKE_ADS_HRESULT(NCERR_KINTRAFO_INVALIDDIM);

			}
		}
	}
	else
	{
		hr = E_POINTER;
	}
	return hr;
}

HRESULT CSixAxisSolver::GetDimensions(ULONG* pFwdInput, ULONG* pFwdOutput)
{
	HRESULT hr = S_OK;

	if (pFwdInput && pFwdOutput)
	{
		*pFwdInput = NumberOfAxes;
		*pFwdOutput = NumberOfAxes;
	}
	else
	{
		hr = E_POINTER; //pointer error
	}
	return hr;
}
///</AutoGeneratedContent>
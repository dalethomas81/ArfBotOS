///////////////////////////////////////////////////////////////////////////////
// RoboticsDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "RoboticsDriver.h"
#include "RoboticsClassFactory.h"

DECLARE_GENERIC_DEVICE(ROBOTICSDRV)

IOSTATUS CRoboticsDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CRoboticsClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CRoboticsDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CRoboticsDriver::ROBOTICSDRV_GetVersion( )
{
	return( (ROBOTICSDRV_Major << 8) | ROBOTICSDRV_Minor );
}


///////////////////////////////////////////////////////////////////////////////
// SixAxisSolverDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "SixAxisSolverDriver.h"
#include "SixAxisSolverClassFactory.h"

DECLARE_GENERIC_DEVICE(SIXAXISSOLVERDRV)

IOSTATUS CSixAxisSolverDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CSixAxisSolverClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CSixAxisSolverDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CSixAxisSolverDriver::SIXAXISSOLVERDRV_GetVersion( )
{
	return( (SIXAXISSOLVERDRV_Major << 8) | SIXAXISSOLVERDRV_Minor );
}


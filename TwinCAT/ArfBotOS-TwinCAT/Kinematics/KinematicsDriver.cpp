///////////////////////////////////////////////////////////////////////////////
// KinematicsDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "KinematicsDriver.h"
#include "KinematicsClassFactory.h"

DECLARE_GENERIC_DEVICE(KINEMATICSDRV)

IOSTATUS CKinematicsDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CKinematicsClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CKinematicsDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CKinematicsDriver::KINEMATICSDRV_GetVersion( )
{
	return( (KINEMATICSDRV_Major << 8) | KINEMATICSDRV_Minor );
}


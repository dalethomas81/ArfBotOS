///////////////////////////////////////////////////////////////////////////////
// TC_DriverDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "TC_DriverDriver.h"
#include "TC_DriverClassFactory.h"

DECLARE_GENERIC_DEVICE(TC_DRIVERDRV)

IOSTATUS CTC_DriverDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CTC_DriverClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CTC_DriverDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CTC_DriverDriver::TC_DRIVERDRV_GetVersion( )
{
	return( (TC_DRIVERDRV_Major << 8) | TC_DRIVERDRV_Minor );
}


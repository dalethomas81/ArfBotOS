///////////////////////////////////////////////////////////////////////////////
// TC_DriverDriver.h

#ifndef __TC_DRIVERDRIVER_H__
#define __TC_DRIVERDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define TC_DRIVERDRV_NAME        "TC_DRIVER"
#define TC_DRIVERDRV_Major       1
#define TC_DRIVERDRV_Minor       0

#define DEVICE_CLASS CTC_DriverDriver

#include "ObjDriver.h"

class CTC_DriverDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl TC_DRIVERDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(TC_DRIVERDRV)
	VxD_Service( TC_DRIVERDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __TC_DRIVERDRIVER_H__
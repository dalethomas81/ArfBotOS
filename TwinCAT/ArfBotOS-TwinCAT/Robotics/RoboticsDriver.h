///////////////////////////////////////////////////////////////////////////////
// RoboticsDriver.h

#ifndef __ROBOTICSDRIVER_H__
#define __ROBOTICSDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define ROBOTICSDRV_NAME        "ROBOTICS"
#define ROBOTICSDRV_Major       1
#define ROBOTICSDRV_Minor       0

#define DEVICE_CLASS CRoboticsDriver

#include "ObjDriver.h"

class CRoboticsDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl ROBOTICSDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(ROBOTICSDRV)
	VxD_Service( ROBOTICSDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __ROBOTICSDRIVER_H__
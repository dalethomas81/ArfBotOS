///////////////////////////////////////////////////////////////////////////////
// KinematicsDriver.h

#ifndef __KINEMATICSDRIVER_H__
#define __KINEMATICSDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define KINEMATICSDRV_NAME        "KINEMATICS"
#define KINEMATICSDRV_Major       1
#define KINEMATICSDRV_Minor       0

#define DEVICE_CLASS CKinematicsDriver

#include "ObjDriver.h"

class CKinematicsDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl KINEMATICSDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(KINEMATICSDRV)
	VxD_Service( KINEMATICSDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __KINEMATICSDRIVER_H__
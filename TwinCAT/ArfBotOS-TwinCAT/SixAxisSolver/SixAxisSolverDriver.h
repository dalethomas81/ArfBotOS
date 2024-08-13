///////////////////////////////////////////////////////////////////////////////
// SixAxisSolverDriver.h

#ifndef __SIXAXISSOLVERDRIVER_H__
#define __SIXAXISSOLVERDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define SIXAXISSOLVERDRV_NAME        "SIXAXISSOLVER"
#define SIXAXISSOLVERDRV_Major       1
#define SIXAXISSOLVERDRV_Minor       0

#define DEVICE_CLASS CSixAxisSolverDriver

#include "ObjDriver.h"

class CSixAxisSolverDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl SIXAXISSOLVERDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(SIXAXISSOLVERDRV)
	VxD_Service( SIXAXISSOLVERDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __SIXAXISSOLVERDRIVER_H__
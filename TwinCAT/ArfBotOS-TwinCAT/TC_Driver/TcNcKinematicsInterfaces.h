///////////////////////////////////////////////////////////////////////////////
// This is a part of the Beckhoff TwinCAT Software Development Kit.
// Copyright (C) Beckhoff Automation GmbH
// All rights reserved.
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "TcInterfaces.h"
#include "TcNcKinematicsServices.h"


/// <summary>
/// Module definition interface to implement custom kinematic transformations.
/// </summary>
#if !defined(_TC_TYPE_05010001_0000_0000_E000_000000000064_INCLUDED_)
#define _TC_TYPE_05010001_0000_0000_E000_000000000064_INCLUDED_
struct __declspec(novtable) ITcNcTrafo : public ITcUnknown
{
	TCOMMETHOD ( Forward(PTcNcTrafoParameter p) );								// i == acs; o == mcs
	TCOMMETHOD ( Backward(PTcNcTrafoParameter p) );								// i == mcs; o == acs
	TCOMMETHOD ( TrafoSupported(PTcNcTrafoParameter p, bool bFwd) );		// check if trafo type is supported (yes = S_OK; no = S_FALSE)
	TCOMMETHOD ( GetDimensions(PULONG pFwdInput, PULONG pFwdOutput) );	// get dimensions before and after (forward) transformation 
};
_TCOM_SMARTPTR_TYPEDEF(ITcNcTrafo, IID_ITcNcTrafo);
#endif // !defined(_TC_TYPE_05010001_0000_0000_E000_000000000064_INCLUDED_)

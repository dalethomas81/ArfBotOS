///////////////////////////////////////////////////////////////////////////////
// This is a part of the Beckhoff TwinCAT Software Development Kit.
// Copyright (C) Beckhoff Automation GmbH
// All rights reserved.
////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "TcServices.h"

#define	SRVNAME_TCNCKINEMATICS	"TcNcKin"
#define	SRVNAME_TCNCKINEMATICSL4 "TcNcKinL4"
const	ULONG DrvID_TcNcKinematics = 0x05010000;

///////////////////////////////////////////////////////////////////////////////
#pragma region Class Ids

#pragma endregion

///////////////////////////////////////////////////////////////////////////////
#pragma region Interface Ids

#if !defined(_TC_IID_05010001_0000_0000_E000_000000000064_INCLUDED_)
#define _TC_IID_05010001_0000_0000_E000_000000000064_INCLUDED_
TCOM_DECL_INTERFACE("05010001-0000-0000-e000-000000000064", ITcNcTrafo) 
#endif // !defined(_TC_IID_05010001_0000_0000_E000_000000000064_INCLUDED_)

#pragma endregion

///////////////////////////////////////////////////////////////////////////////
#pragma region Category Ids

#pragma endregion

///////////////////////////////////////////////////////////////////////////////
#pragma region Parameter Ids

#pragma endregion

///////////////////////////////////////////////////////////////////////////////
#pragma region Error Ids

// kinematics transformation errors
const long NCERR_KINTRAFO_OFFSET                                   = 0x00;
const long NCERR_KINTRAFO_FAILED                                   = ERR_NCKINERRS + NCERR_KINTRAFO_OFFSET + 0x00; // 0x4C00
const long NCERR_KINTRAFO_AMBIGUOUSSOLUTION                        = ERR_NCKINERRS + NCERR_KINTRAFO_OFFSET + 0x01; // 0x4C01
const long NCERR_KINTRAFO_INVALIDAXISPOS                           = ERR_NCKINERRS + NCERR_KINTRAFO_OFFSET + 0x02; // 0x4C02
const long NCERR_KINTRAFO_INVALIDDIM                               = ERR_NCKINERRS + NCERR_KINTRAFO_OFFSET + 0x03; // 0x4C03

#pragma endregion

///////////////////////////////////////////////////////////////////////////////
#pragma region Enumerations

#if !defined(_TC_TYPE_D400B256_8F0F_48BA_B147_2825944B2C33_INCLUDED_)
#define _TC_TYPE_D400B256_8F0F_48BA_B147_2825944B2C33_INCLUDED_
enum	EcNcTrafoParameter
{
	EcNcTrafoParameter_Invalid	= 0,
	EcNcTrafoParameter_Base		= 1,
	EcNcTrafoParameter_Ext		= 2,
	EcNcTrafoParameter_ExtCnc	= 3,
};
#endif // !defined(_TC_TYPE_D400B256_8F0F_48BA_B147_2825944B2C33_INCLUDED_)

#pragma endregion

///////////////////////////////////////////////////////////////////////////////
#pragma region Type Definitions

#if !defined(_TC_TYPE_082AF37D_C470_42C2_A856_80CD5C30114A_INCLUDED_)
#define _TC_TYPE_082AF37D_C470_42C2_A856_80CD5C30114A_INCLUDED_
struct TcNcTrafoParameter
{
	TcNcTrafoParameter() 
		: type(EcNcTrafoParameter_Base)
		, i(NULL), d_i(NULL), dd_i(NULL)
		, o(NULL), d_o(NULL), dd_o(NULL)
		, dim_i(0), dim_o(0), dim_para(0)		
		, para(NULL), torque(NULL), payload(0.0)
		, tool_len(0.0)
	{
	}

	EcNcTrafoParameter	type;
	ULONG						dim_i;		// dim of input vectors (i, d_i, dd_i)
	ULONG						dim_o;		// dim of output vectors (o, d_o, dd_o, torque)
	ULONG						dim_para;	// dim of additional parameter (para)

	const double*			i;				// input values parameter (dim_i)
	const double*			d_i;		
	const double*			dd_i;

	double*					o;				// output values parameter (dim_i)
	double*					d_o;
	double*					dd_o;

	double*					torque;
	const double*			para;			// additional parameter (dim_p)

	double					payload;		// weight in kg
	double					tool_len;	// actual tool length in [mm]
};
typedef TcNcTrafoParameter *PTcNcTrafoParameter;
#endif // !defined(_TC_TYPE_082AF37D_C470_42C2_A856_80CD5C30114A_INCLUDED_)

struct TcNcTrafoParameterExt : public TcNcTrafoParameter
{
	PVOID	info;
	ULONG	infoSize;
	ULONG	infoType;
};
typedef TcNcTrafoParameterExt *PTcNcTrafoParameterExt;

#pragma endregion

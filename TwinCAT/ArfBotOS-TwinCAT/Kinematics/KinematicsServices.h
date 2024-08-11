///////////////////////////////////////////////////////////////////////////////
// KinematicsServices.h

#pragma once

#include "TcServices.h"

const ULONG DrvID_Kinematics = 0x3F000000;
#define SRVNAME_KINEMATICS "Kinematics"

///<AutoGeneratedContent id="ClassIDs">
const CTCID CID_KinematicsCKinematicsModule = {0xc3311bcc,0x75c8,0x4482,{0x9b,0xdc,0x69,0x9f,0x38,0xd2,0x88,0xe2}};
///</AutoGeneratedContent>

///<AutoGeneratedContent id="DataAreaIDs">
#define ADI_KinematicsModuleInputs 0
#define ADI_KinematicsModuleOutputs 1
///</AutoGeneratedContent>

///<AutoGeneratedContent id="ParameterIDs">
const PTCID PID_KinematicsModuleParameter = 0x00000001;
///</AutoGeneratedContent>

///<AutoGeneratedContent id="InterfaceIDs">
///</AutoGeneratedContent>

///<AutoGeneratedContent id="DataTypes">
typedef struct _KinematicsModuleParameter
{
	ULONG data1;
	ULONG data2;
	double data3;
} KinematicsModuleParameter, *PKinematicsModuleParameter;

typedef struct _KinematicsModuleInputs
{
	ULONG Value;
	ULONG Status;
	ULONG Data;
} KinematicsModuleInputs, *PKinematicsModuleInputs;

typedef struct _KinematicsModuleOutputs
{
	ULONG Value;
	ULONG Control;
	ULONG Data;
} KinematicsModuleOutputs, *PKinematicsModuleOutputs;

///</AutoGeneratedContent>



///<AutoGeneratedContent id="EventClasses">
///</AutoGeneratedContent>
FUNCTION_BLOCK FB_GroupHandler
VAR_IN_OUT
	AxisGroup: AXIS_GROUP_REF_SM3;
END_VAR
VAR_INPUT
	Enable: BOOL;
	StoredPositions: REFERENCE TO ARRAY[GVL_App.POS_ARR_BEG..GVL_App.POS_ARR_END] OF DUT_StoredPosition;
END_VAR
VAR_OUTPUT
	Busy: BOOL;
	Error: BOOL;
	ErrorID: SMC_ERROR;
END_VAR
VAR
	//
	EnableFtrig: F_TRIG;
	
	//
	_SeqEnableGroup: INT;
	_SeqDisableGroup: INT;
	_SequenceSetKinematics: INT;
	_SequenceSetCoordTrans: INT;
	_SeqResetGroup: INT;
	
	//
	_SMC_GroupPower: SMC_GroupPower;
	_GroupPower: BOOL;
	_MC_GroupEnable: MC_GroupEnable;
	_MC_GroupDisable: MC_GroupDisable;
	_MC_GroupReset: MC_GroupReset;
	_MC_GroupReadActualPosition: MC_GroupReadActualPosition;
	_SMC_SetKinConfiguration: SMC_SetKinConfiguration;
	
    // coordinate transforms
	_MC_SetCoordinateTransform: MC_SetCoordinateTransform;
	_SMC_GroupSetTool: SMC_GroupSetTool;
	_MC_COORD_REF: MC_COORD_REF;
	_CoordinateSystem: SMC_COORD_SYSTEM:=SMC_COORD_SYSTEM.ACS;
	
	_EnableGroup: BOOL;
	_DisableGroup: BOOL;
	_GroupEnabled: BOOL;
	_ResetGroup: BOOL;
	_SetKinematics: BOOL;
	_SetCoordinates: BOOL;
	_KinematicsSet: BOOL;
	
	_MC_GroupReadStatus: MC_GroupReadStatus;
	_DUT_GroupStatus: DUT_GroupStatus;
	
END_VAR

VAR CONSTANT
	STEP_DONE: 				INT := 1000;
	STEP_IDLE: 				INT := 0000;
	STEP_GROUP_POWER_ON: 	INT := 0022;
	STEP_WAIT_POWER_ON: 	INT := 0024;
	STEP_GROUP_ENABLE: 		INT := 0030;
	STEP_WAIT_ENABLE: 		INT := 0035;
	STEP_READ_POSITION: 	INT := 0040;
	STEP_WAIT_POSITION: 	INT := 0050;
	STEP_SET_KINEMATICS: 	INT := 0060;
	STEP_WAIT_KINEMATICS: 	INT := 0070;
	STEP_SET_COORD_TRANS: 	INT := 0071;
	STEP_WAIT_COORD_TRANS: 	INT := 0072;
	STEP_GROUP_DISABLE: 	INT := 0100;
	STEP_WAIT_DISABLE: 		INT := 0110;
	STEP_GROUP_POWER_OFF: 	INT := 0120;
END_VAR

//
Busy := _SeqEnableGroup <> 0
		OR _SeqDisableGroup <> 0
		OR _SequenceSetKinematics <> 0
		OR _SequenceSetCoordTrans <> 0
		OR _SeqResetGroup <> 0;
		
EnableFtrig(CLK:=Enable);
IF EnableFtrig.Q THEN
	M_DisableGroup();
END_IF

//
_M_EnableGroup();
_M_DisableGroup();
_M_SetKinematics();
_M_ResetGroup();

_MC_GroupReadStatus(
	AxisGroup:= AxisGroup, 
	Enable:= Enable, 
	Valid=> _DUT_GroupStatus.Valid, 
	Busy=> _DUT_GroupStatus.Busy, 
	Error=> _DUT_GroupStatus.Error, 
	ErrorID=> _DUT_GroupStatus.ErrorID, 
	GroupMoving=> _DUT_GroupStatus.GroupMoving, 
	GroupHoming=> _DUT_GroupStatus.GroupHoming, 
	GroupErrorStop=> _DUT_GroupStatus.GroupErrorStop, 
	GroupStandby=> _DUT_GroupStatus.GroupStandby, 
	GroupStopping=> _DUT_GroupStatus.GroupStopping, 
	GroupDisabled=> _DUT_GroupStatus.GroupDisabled, 
	TrackingDynamicCS=> _DUT_GroupStatus.TrackingDynamicCS, 
	InSync=> _DUT_GroupStatus.InSync, 
	ActiveMovementId=> _DUT_GroupStatus.ActiveMovementId, 
	FirstReferencedMovementId=> _DUT_GroupStatus.FirstReferencedMovementId, 
	LastAcceptedMovementId=> _DUT_GroupStatus.LastAcceptedMovementId, 
	PositionLag=> _DUT_GroupStatus.PositionLag, 
	OrientationLag=> _DUT_GroupStatus.OrientationLag);

_MC_SetCoordinateTransform(
	AxisGroup:= AxisGroup, 
	Execute:= , 
	CoordTransform:= _MC_COORD_REF, 
	CoordSystem:= , 
	Done=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );

_SMC_GroupSetTool(
	AxisGroup:= AxisGroup, 
	Execute:= , 
	ToolOffset:= _MC_COORD_REF, 
	Done=> , 
	Error=> , 
	ErrorID=> );

_SMC_GroupPower(
	AxisGroup:= AxisGroup, 
	Enable:= _GroupPower, 
	bRegulatorOn:= _GroupPower, 
	bDriveStart:= _GroupPower, 
	Status=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );

_MC_GroupEnable(
	AxisGroup:= AxisGroup, 
	Execute:= , 
	CompatibilityOptions:= , 
	Done=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );
	
_MC_GroupDisable(
	AxisGroup:= AxisGroup, 
	Execute:= , 
	Done=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );

_MC_GroupReset(
	AxisGroup:= AxisGroup, 
	Execute:= , 
	Done=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );

_MC_GroupReadActualPosition(
	AxisGroup:= AxisGroup, 
	Enable:= , 
	CoordSystem:= _CoordinateSystem, 
	Valid=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> , 
	Position=> , 
	KinematicConfig=> );
	
_SMC_SetKinConfiguration(
	AxisGroup:= AxisGroup, 
	Execute:= , 
	ConfigData:= _MC_GroupReadActualPosition.KinematicConfig, 
	Done=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );
END_FUNCTION_BLOCK

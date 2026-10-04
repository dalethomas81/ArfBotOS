FUNCTION_BLOCK FB_JogProcessor
VAR_IN_OUT
	AxisGroup: AXIS_GROUP_REF_SM3;
	GroupHandler: FB_GroupHandler;
	Controller: FB_Controller;
END_VAR
VAR_INPUT
	Enable: BOOL;
	Reset: BOOL;
	StoredPositions: REFERENCE TO ARRAY[GVL_App.POS_ARR_BEG..GVL_App.POS_ARR_END] OF DUT_StoredPosition;
	VisionRegisters: REFERENCE TO ARRAY[GVL_App.VIS_ARR_BEG..GVL_App.VIS_ARR_END] OF DUT_VisionRegister;
	Flags: REFERENCE TO ARRAY[GVL_App.FLG_ARR_BEG..GVL_App.FLG_ARR_END] OF DUT_Flags;
END_VAR
VAR_OUTPUT
	ABC_as_ACS: BOOL;
	CoordSystem: SMC_COORD_SYSTEM;
	Position: SMC_POS_REF;
	Enabled: BOOL;
	Done: BOOL;
	Busy: BOOL;
	Error: BOOL;
	ErrorID: SMC_ERROR;
END_VAR
VAR
	_ExecuteRtrig: R_TRIG;
	_ExecuteFtrig: F_TRIG;
	_ResetRtrig: R_TRIG;
	_ErrorRtrig: R_TRIG;
	_EnableFtrig: F_TRIG;
	
	_SequenceMain: INT;
	_SequenceSetCoordTrans: INT;
	
	_SMC_GroupJog2: SMC_GroupJog2;
	
	_ControllerCenter: USINT:=128;
	_ControllerDeadZone: USINT:=40;
	
	_OffsetExist: ARRAY[0..GVL_App.MCS_POS_IDX-GVL_App.TCP_POS_IDX] OF BOOL;
	
	_MoveToPose: ARRAY[0..GVL_App.POSE_5_IDX-GVL_App.POSE_1_IDX] OF BOOL;
	_MoveCommandProcessor: FB_MoveCmdProcessor;
	
	_i, _j, _k: INT;
	_FirstScan: BOOL:=TRUE;
	
END_VAR

VAR CONSTANT
	STEP_DONE: 				INT := 1000;
	STEP_IDLE: 				INT := 0000;
	STEP_GROUP_ENABLE: 		INT := 0030;
	STEP_WAIT_ENABLE: 		INT := 0035;
	STEP_SET_KINEMATICS: 	INT := 0060;
	STEP_WAIT_KINEMATICS: 	INT := 0070;
	STEP_JOG:				INT := 0080;
	STEP_WAIT_JOG:			INT := 0090;
	STEP_GROUP_DISABLE: 	INT := 0100;
	STEP_WAIT_DISABLE: 		INT := 0110;
END_VAR

_ExecuteRtrig(CLK:=_SMC_GroupJog2.Enable);
_ExecuteFtrig(CLK:=_SMC_GroupJog2.Enable);
_ResetRtrig(CLK:=Reset);

IF _FirstScan THEN
	_SMC_GroupJog2.Velocity := 50.0;
	_SMC_GroupJog2.Acceleration := 300.0;
	_SMC_GroupJog2.Deceleration := 300.0;
	_SMC_GroupJog2.Jerk := 1000.0;
	_SMC_GroupJog2.MaxAngularDistance := 90.0;
	_SMC_GroupJog2.MaxLinearDistance := 200.0;
	_SMC_GroupJog2.AccFactor := 1.0;
	_SMC_GroupJog2.VelFactor := 0.1;
	_SMC_GroupJog2.JerkFactor := 1.0;
	_SMC_GroupJog2.TorqueFactor := 1.0;
	_SMC_GroupJog2.CoordSystem := SMC_COORD_SYSTEM.TCS;
	
END_IF

//
_M_HandleController(Controller:=Controller);
_M_HandlePose(AxisGroup:=AxisGroup, GroupHandler:=GroupHandler);

//
_EnableFtrig(CLK:=Enable);
IF _EnableFtrig.Q THEN
	_SMC_GroupJog2.Enable := FALSE;
END_IF

//
CASE _SequenceMain OF
	STEP_DONE: // done
		Busy := FALSE;
		Done := TRUE;
		_SequenceMain := STEP_IDLE;
		_MoveCommandProcessor.M_Home();
		
	STEP_IDLE: // IDLE
		IF _ExecuteRtrig.Q AND Enable THEN
			Busy := TRUE;
			Done := FALSE;
			_M_CheckOffsets();
			_SequenceMain := STEP_GROUP_ENABLE;
		END_IF
		
	STEP_GROUP_ENABLE:
		IF GroupHandler.M_EnableGroup() THEN
			_SequenceMain := STEP_WAIT_ENABLE;
		ELSE
			_SequenceMain := STEP_DONE;
			Error := TRUE;
			ErrorID := GroupHandler.ErrorID;
		END_IF
		
	STEP_WAIT_ENABLE:
		// TODO add timeout timer here
		IF GroupHandler.P_GroupEnabled THEN
			_SequenceMain := STEP_SET_KINEMATICS;
		END_IF;
		
	STEP_SET_KINEMATICS: // set kinematics
        // orientation convention is always ZYZ for softmotion
        GVL_App.OriConvention := SMC_ORI_CONVENTION.ZYZ;
        //
		IF GroupHandler.M_SetKinematics(_SMC_GroupJog2.CoordSystem) THEN
			_SequenceMain := STEP_WAIT_KINEMATICS;
		ELSE
			_SequenceMain := STEP_DONE;
			Error := TRUE;
			ErrorID := GroupHandler.ErrorID;
		END_IF
		
	STEP_WAIT_KINEMATICS:
		// TODO add timeout timer here
		IF GroupHandler.P_KinematicsSet THEN
			_SequenceMain := STEP_WAIT_JOG;
		END_IF
		
	STEP_WAIT_JOG: //
		IF _SMC_GroupJog2.Error THEN
			Error := TRUE;
			ErrorID := _SMC_GroupJog2.ErrorID;
		END_IF
		IF _ExecuteFtrig.Q THEN
			//_SequenceMain := STEP_GROUP_DISABLE;
			_SequenceMain := STEP_DONE;
		END_IF
		
	// TODO: do we need to disable the group at this point? lets leave this here for now but skip it.	
	STEP_GROUP_DISABLE:
		IF GroupHandler.M_DisableGroup() THEN
			_SequenceMain := STEP_WAIT_DISABLE;
		ELSE
			_SequenceMain := STEP_DONE;
			Error := TRUE;
			ErrorID := GroupHandler.ErrorID;
		END_IF
		
	STEP_WAIT_DISABLE:
		// TODO add timeouttimer here
		IF NOT GroupHandler.P_GroupEnabled THEN
			_SequenceMain := STEP_DONE;
		END_IF
	
END_CASE

//
IF _ResetRtrig.Q AND Error THEN
	Error := FALSE;
	ErrorID := SMC_NO_ERROR;
	_SequenceMain := STEP_DONE;
	GroupHandler.M_ResetGroup();
	_MoveCommandProcessor.M_Home();
END_IF

_SMC_GroupJog2(
	AxisGroup:= AxisGroup, 
	Enable:= , 
	Forward:= , 
	Backward:= , 
	MaxLinearDistance:= , 
	MaxAngularDistance:= , 
	Velocity:= , 
	Acceleration:= , 
	Deceleration:= , 
	Jerk:= , 
	VelFactor:= , 
	AccFactor:= , 
	JerkFactor:= , 
	TorqueFactor:= , 
	CoordSystem:= , 
	ABC_as_ACS:= , 
	Active=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> , 
	CurrentPosition=> Position);

//
CoordSystem := _SMC_GroupJog2.CoordSystem;
ABC_as_ACS := _SMC_GroupJog2.ABC_as_ACS;
Enabled := _SMC_GroupJog2.Enable;

//
_ErrorRtrig(CLK:=GroupHandler.Error
				OR _SMC_GroupJog2.Error
				OR _MoveCommandProcessor.Error);			
IF _ErrorRtrig.Q THEN
	// TODO need to rework how error messages are handled (globally)
	// if the move command processor has an error, we dont know what it is.
	Error := TRUE;
END_IF

// do not put anything after this
_FirstScan := FALSE;
END_FUNCTION_BLOCK

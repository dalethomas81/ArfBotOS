FUNCTION_BLOCK FB_CommandProcessor EXTENDS FB_Module
VAR_IN_OUT
	J1, J2, J3, J4, J5, J6: AXIS_REF_SM3;
	AxisGroup: AXIS_GROUP_REF_SM3;
	GroupHandler: FB_GroupHandler;
	CmdIO: ST_LogicCmdIO;
	PrgProcessor: FB_ProgramProcessor;
END_VAR
VAR_INPUT
	Enable: BOOL;
END_VAR
VAR_OUTPUT
	ItfActiveProcessorUnitMode: PACK_ML.IUnitMode;
	ItfActiveModule: ItfModule;
	StatusText: STRING(255);
    GCodeLines: ARRAY[0..SMC_GCodeViewer.c_uiLines] OF STRING;
END_VAR
VAR
	_Commands: ARRAY [GVL_App.PROG_CMDS_ARR_BEG..GVL_App.PROG_CMDS_ARR_END] OF DUT_Command;
	_DummyUnitMode: PACK_ML.UnitMode;
	_UnitModeCurrentState: PACK_ML.State;
	_ActiveCommand: DUT_Command;
	_CurrentCmdIdx: UINT;
	_ProcessMoveCommand: FB_MoveCmdProcessor;
	_ProcessPauseCommand: FB_PauseCmdProcessor;
	//_ProcessInputCommand: InputCommandProcessor;
	_ProcessOutputCommand: FB_OutputCmdProcessor;
	_ProcessorDoneRtrig: R_TRIG;
    _ProcessVisionCommand: REFERENCE TO FB_VisionCmdProcessor; // keep vision in separate task
	_ProcessVarSetCommand: FB_VarSetCmdProcessor;
	_ProcessCncCommand: FB_CncCmdProcessor;
	_ProcessLogicCommand: FB_LogicCmdProcessor;
    _ErrorRtrig: ARRAY[0..7] OF R_TRIG;
	_FirstScan: BOOL:=TRUE;
END_VAR

VAR CONSTANT
	_STEP_DONE: 			INT := 1000;
	_STEP_IDLE: 			INT := 0000;
	_STEP_CMD:				INT := 0010;
	_STEP_WAIT:				INT := 0020;
	_STEP_ERR:              INT := 0030;
END_VAR

// keep this reference at the top in case we reference this somewhere else first
// hint: originally this was an issue because of the done and reset bits being referenced first
_ProcessVisionCommand REF= _40_Vision.ProcessVisionCommand;

//
SUPER^();

//
IF _FirstScan THEN
	// will exception if we don't set this on first scan
	_DummyUnitMode(sName := 'Active Processor', 
					eInitialState := PACK_ML.State.Idle,
					xActive := TRUE);
	ItfActiveProcessorUnitMode := _DummyUnitMode;
END_IF

// this is at best a hack and needs to be rethought out.
// we logically cant assign a process as the active unit mode 
// at start up so we use this dummy one
CASE _DummyUnitMode.CurrentState OF
	PACK_ML.State.Stopped,
	PACK_ML.State.Aborted,
	PACK_ML.State.Held,
	PACK_ML.State.Suspended,
	PACK_ML.State.Complete,
	PACK_ML.State.Idle:
		//
	PACK_ML.State.Resetting,
	PACK_ML.State.Starting,
	PACK_ML.State.Execute,
	PACK_ML.State.Suspending,
	PACK_ML.State.UnSuspending,
	PACK_ML.State.Holding,
	PACK_ML.State.UnHolding,
	PACK_ML.State.Completing,
	PACK_ML.State.Aborting,
	PACK_ML.State.Stopping,
	PACK_ML.State.Clearing:
		_DummyUnitMode.ActingStateCompleted();
END_CASE

// PackML state machine implementation
_UnitModeCurrentState := SUPER^._UnitMode.CurrentState;

//
_ErrorRtrig[0](CLK:=_ProcessMoveCommand.Error);
IF _ErrorRtrig[0].Q THEN
	SUPER^.M_Error(Description:=CONCAT('Move Error: ', _ProcessMoveCommand.ErrorDescription));
	StatusText:=SUPER^.ErrorDescription;
END_IF
_ErrorRtrig[1](CLK:=_ProcessPauseCommand.Error);
IF _ErrorRtrig[1].Q THEN
	SUPER^.M_Error(Description:=CONCAT('Pause Error: ', _ProcessPauseCommand.ErrorDescription));
	StatusText:=SUPER^.ErrorDescription;
END_IF
(*
_ErrorRtrig[2](CLK:=_ProcessInputCommand.Error);
IF _ErrorRtrig[2].Q THEN
	SUPER^._M_Error(Description:=CONCAT('Input Error: ', _ProcessInputCommand.ErrorDescription));
	StatusText:=SUPER^.ErrorDescription;
END_IF*)
_ErrorRtrig[3](CLK:=_ProcessOutputCommand.Error);
IF _ErrorRtrig[3].Q THEN
	SUPER^.M_Error(Description:=CONCAT('Output Error: ', _ProcessOutputCommand.ErrorDescription));
	StatusText:=SUPER^.ErrorDescription;
END_IF 
_ErrorRtrig[4](CLK:=_ProcessVisionCommand.Error);      
IF _ErrorRtrig[4].Q THEN
	SUPER^.M_Error(Description:=CONCAT('Vision Error: ', _ProcessVisionCommand.ErrorDescription));
	StatusText:=SUPER^.ErrorDescription;
END_IF
_ErrorRtrig[5](CLK:=_ProcessVarSetCommand.Error);
IF _ErrorRtrig[5].Q THEN
	SUPER^.M_Error(Description:=CONCAT('Variable Error: ', _ProcessVarSetCommand.ErrorDescription));
	StatusText:=SUPER^.ErrorDescription;
END_IF
_ErrorRtrig[6](CLK:=_ProcessCncCommand.Error);
IF _ErrorRtrig[6].Q THEN
	SUPER^.M_Error(Description:=CONCAT('CNC Error: ', _ProcessCncCommand.ErrorDescription));
	StatusText:=SUPER^.ErrorDescription;
END_IF
_ErrorRtrig[7](CLK:=_ProcessLogicCommand.Error);
IF _ErrorRtrig[7].Q THEN
	SUPER^.M_Error(Description:=CONCAT('Logic Error: ', _ProcessLogicCommand.ErrorDescription));
	StatusText:=SUPER^.ErrorDescription;
END_IF

_ProcessMoveCommand(
	StoredPositions:= SUPER^.StoredPositions,
	VisionRegisters:= SUPER^.VisionRegisters,
	Flags:= SUPER^.Flags,
	AxisGroup:= AxisGroup,
	GroupHandler:= GroupHandler,
	Velocity:= PersistentVars.StoredPositions[GVL_App.SPEEDS_IDX].Position.X, 
	VelFactor:= PersistentVars.StoredPositions[GVL_App.SPEEDS_IDX].Position.A,
	Acceleration:= PersistentVars.StoredPositions[GVL_App.SPEEDS_IDX].Position.Y, 
	AccFactor:= PersistentVars.StoredPositions[GVL_App.SPEEDS_IDX].Position.B,
	Jerk:= PersistentVars.StoredPositions[GVL_App.SPEEDS_IDX].Position.Z,
	JerkFactor:= PersistentVars.StoredPositions[GVL_App.SPEEDS_IDX].Position.C, 
	Position=> ,
	ItfUnitMode=>, 
	Error=> );

_ProcessPauseCommand(
	StoredPositions:= SUPER^.StoredPositions,
	VisionRegisters:= SUPER^.VisionRegisters,
	Flags:= SUPER^.Flags, 
	ElapsedTime=> , 
	RemainingTime=> , 
	ItfUnitMode=>, 
	Error=> );

//_ProcessInputCommand();
_ProcessOutputCommand(
	StoredPositions:= SUPER^.StoredPositions,
	VisionRegisters:= SUPER^.VisionRegisters,
	Flags:= SUPER^.Flags,
	Digital:= CmdIO.Output,
	Pwm:= CmdIO.Pwm,
	Enable:= Enable,
	ItfUnitMode=>, 
	Error=> );
	
_ProcessVarSetCommand( 
	StoredPositions:= SUPER^.StoredPositions,
	VisionRegisters:= SUPER^.VisionRegisters,
	Flags:= SUPER^.Flags,
	ItfUnitMode=>, 
	Error=> , 
	ErrorDescription=> );

_ProcessCncCommand(
	StoredPositions:= SUPER^.StoredPositions,
	VisionRegisters:= SUPER^.VisionRegisters,
	Flags:= SUPER^.Flags,
	J1:=J1, J2:=J2, J3:=J3, J4:=J4, J5:=J5, J6:=J6,
	AxisGroup:= AxisGroup, 
    CmdIO:= CmdIO,
    GCodeLines => GCodeLines,
	GapJerk:= , 
	GapVelocity:= , 
	AvoidGaps:= , 
	GapAccel:= , 
	ItfUnitMode=>, 
	Error=> , 
	ErrorDescription=> );
	
_ProcessLogicCommand(
	StoredPositions:= SUPER^.StoredPositions,
	VisionRegisters:= SUPER^.VisionRegisters,
	Flags:= SUPER^.Flags,
	CmdProcessor:= THIS^,
	PrgProcessor:= PrgProcessor,
	ItfUnitMode=>, 
	Error=> , 
	ErrorDescription=> , 
	CmdIO:= CmdIO);
	
_FirstScan := FALSE;
END_FUNCTION_BLOCK

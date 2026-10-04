FUNCTION_BLOCK FB_ProgramProcessor EXTENDS FB_Module
VAR_IN_OUT
	Programs: ARRAY [GVL_App.PROG_ARR_BEG..GVL_App.PROG_ARR_END] OF DUT_Program;
	J1, J2, J3, J4, J5, J6: AXIS_REF_SM3;
	AxisGroup: AXIS_GROUP_REF_SM3;
	GroupHandler: FB_GroupHandler;
	CmdIO: ST_LogicCmdIO;
END_VAR
VAR_INPUT
	Enable: BOOL;
END_VAR
VAR_OUTPUT
	ItfActiveProcessorUnitMode	: PACK_ML.IUnitMode;
	ItfActiveModule: ItfModule;
	StatusText: STRING(255);
    GCodeLines: ARRAY[0..SMC_GCodeViewer.c_uiLines] OF STRING;
END_VAR
VAR
	_DummyUnitMode: PACK_ML.UnitMode;
	_UnitModeCurrentState: PACK_ML.State;
	_Returning:  ARRAY [GVL_App.PROG_ARR_BEG..GVL_App.PROG_ARR_END] OF BOOL;
	_ReturnPrograms: ARRAY [GVL_App.PROG_ARR_BEG..GVL_App.PROG_ARR_END] OF UINT;
	_ReturnCommands: ARRAY [GVL_App.PROG_CMDS_ARR_BEG..GVL_App.PROG_CMDS_ARR_END + 1] OF UINT; // +1 because we want to be able to tell if calling program was on last command
	_CurrentPrgIdx: UINT;
	_ProcessCommands: FB_CommandProcessor;
	_Commands: ARRAY [GVL_App.PROG_CMDS_ARR_BEG..GVL_App.PROG_CMDS_ARR_END] OF DUT_Command;
    _ErrorRtrig: R_TRIG;
	_i, _j, _k: INT;
	_FirstScan: BOOL:=TRUE;
END_VAR

VAR CONSTANT
	_STEP_DONE: 			INT := 1000;
	_STEP_IDLE: 			INT := 0000;
	_STEP_PRG:				INT := 0010;
	_STEP_WAIT_EXECUTE:		INT := 0020;
	_STEP_WAIT_IDLE:		INT := 0025;
	_STEP_ERR:              INT := 0030;
END_VAR

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
_ErrorRtrig(CLK:=_ProcessCommands.Error);
IF _ErrorRtrig.Q THEN
	SUPER^.M_Error(Description:=CONCAT('Command Processor Error: ', _ProcessCommands.ErrorDescription));
	StatusText:=SUPER^.ErrorDescription;
END_IF

//
_ProcessCommands(
	StoredPositions:=SUPER^.StoredPositions,
	VisionRegisters:=SUPER^.VisionRegisters,
	Flags:=SUPER^.Flags,
	J1:=J1, J2:=J2, J3:=J3, J4:=J4, J5:=J5, J6:=J6,
	AxisGroup:= AxisGroup,
	GroupHandler:= GroupHandler,
	CmdIO:=CmdIO,
	PrgProcessor:= THIS^,
    GCodeLines => GCodeLines,
	Enable:= Enable,
	Error=> ,
	StatusText=>StatusText);	

_FirstScan := FALSE;
END_FUNCTION_BLOCK

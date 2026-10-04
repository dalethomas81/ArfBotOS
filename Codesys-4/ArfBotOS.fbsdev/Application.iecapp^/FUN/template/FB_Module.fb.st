FUNCTION_BLOCK FB_Module EXTENDS FB_PackML IMPLEMENTS ItfModule

VAR_INPUT // input vars
	StoredPositions: REFERENCE TO ARRAY[GVL_App.POS_ARR_BEG..GVL_App.POS_ARR_END] OF DUT_StoredPosition;
	VisionRegisters: REFERENCE TO ARRAY[GVL_App.VIS_ARR_BEG..GVL_App.VIS_ARR_END] OF DUT_VisionRegister;
	Flags: REFERENCE TO ARRAY[GVL_App.FLG_ARR_BEG..GVL_App.FLG_ARR_END] OF DUT_Flags;
END_VAR

VAR_OUTPUT // output vars
	ItfUnitMode: PACK_ML.IUnitMode;
	Busy: BOOL;
	Error: BOOL;
	ErrorDescription: STRING;
END_VAR

VAR // standard local vars
	_TimeoutTimer: TON:=(PT:=T#0MS);
	_Home, _Start: BOOL;
	_ModuleFirstScan: BOOL:=TRUE;
	_SeqStart, _SeqHome: INT;
	_AbortedRtrig: R_Trig;
END_VAR

VAR CONSTANT //
	STEP_IDLE: INT := 0000;
	STEP_DONE: INT := 0020;
	STEP_RUN:  INT := 0030;
END_VAR

SUPER^();

//
IF _ModuleFirstScan THEN
	ItfUnitMode := SUPER^.IUnitMode;
END_IF;

//
_M_Home();
_M_Start();

//
Busy := _UnitMode.CurrentState <> PACK_ML.State.Idle
		AND _UnitMode.CurrentState <> PACK_ML.State.Aborted
		AND _UnitMode.CurrentState <> PACK_ML.State.Complete
		AND _UnitMode.CurrentState <> PACK_ML.State.Held
		AND _UnitMode.CurrentState <> PACK_ML.State.Stopped
		AND _UnitMode.CurrentState <> PACK_ML.State.Suspended;
		
//
_AbortedRtrig(CLK:=_UnitMode.CurrentState = PACK_ML.State.Aborted);
IF _AbortedRtrig.Q THEN
	//_SeqStart := 0;  _Start:=FALSE;
	//_SeqHome := 0; _Home:=FALSE;
END_IF

//
_TimeoutTimer(IN:= Busy AND _TimeoutTimer.PT > T#0MS); // ignore if timer set to 0
IF _TimeoutTimer.Q THEN
	M_Error(Description:= CONCAT('Task timed out after ', TIME_TO_STRING(_TimeoutTimer.ET)));
END_IF

// do not place anything below this
_ModuleFirstScan:=FALSE;
END_FUNCTION_BLOCK

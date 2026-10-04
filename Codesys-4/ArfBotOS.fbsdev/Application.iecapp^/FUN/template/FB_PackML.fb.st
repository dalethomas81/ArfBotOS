{attribute 'reflection'}
FUNCTION_BLOCK FB_PackML
VAR_INPUT
END_VAR
VAR_OUTPUT
	IUnitMode: PACK_ML.IUnitMode;
END_VAR
VAR
	 {attribute 'instance-path'}
	 {attribute 'noinit'}
	 instance_path: STRING(255);
	_UnitMode: PACK_ML.UnitMode;
	_StateTimer: TON;
	_PackMLFirstScan: BOOL:=TRUE;
	_SeqMain, _SeqAborting, _SeqClearing, _SeqCompleting, _SeqHolding, _SeqResetting,
	_SeqStarting, _SeqStopping, _SeqSuspending, _SeqUnholding, _SeqUnSuspending: INT;
END_VAR
VAR CONSTANT
	STATE_TIME: TIME := T#0MS;
END_VAR

//
IF _PackMLFirstScan THEN
	_PackMLFirstScan := FALSE;
	M_SetPackMl();
	IUnitMode := _UnitMode;
END_IF

//
_StateTimer(PT:=STATE_TIME);

//
CASE _UnitMode.CurrentState OF
	PACK_ML.State.Stopped:
		M_Stopped();
	PACK_ML.State.Aborted:
		M_Aborted();
	PACK_ML.State.Held:
		M_Held();
	PACK_ML.State.Suspended:
		M_Suspended();
	PACK_ML.State.Complete:
		M_Complete();
	PACK_ML.State.Idle:
		M_Idle();
	PACK_ML.State.Resetting:
		M_Resetting();
	PACK_ML.State.Starting:
		M_Starting();
	PACK_ML.State.Execute:
		M_Execute();
	PACK_ML.State.Suspending:
		M_Suspending();
	PACK_ML.State.UnSuspending:
		M_UnSuspending();
	PACK_ML.State.Holding:
		M_Holding();
	PACK_ML.State.UnHolding:
		M_UnHolding();
	PACK_ML.State.Completing:
		M_Completing();
	PACK_ML.State.Aborting:
		M_Aborting();
	PACK_ML.State.Stopping:
		M_Stopping();
	PACK_ML.State.Clearing:
		M_Clearing();
END_CASE
END_FUNCTION_BLOCK

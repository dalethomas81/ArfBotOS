METHOD PRIVATE _M_Start : BOOL

CASE _SeqStart OF
	0:
		IF _Start THEN
			_Start := FALSE;
			M_Reset();
			_SeqStart := _SeqStart + 1;
		END_IF
	1:
		CASE _UnitMode.CurrentState OF
			PACK_ML.State.Clearing,
			PACK_ML.State.Resetting,
			PACK_ML.State.UnHolding,
			PACK_ML.State.UnSuspending,
			PACK_ML.State.Starting:
				// be patient
			PACK_ML.State.Aborted:
				_UnitMode.Clear();
			PACK_ML.State.Stopped:
				_UnitMode.Reset();
			PACK_ML.State.Held:
				_UnitMode.UnHold();
			PACK_ML.State.Suspended:
				_UnitMode.UnSuspend();
			PACK_ML.State.Idle:
				_UnitMode.Start();
			PACK_ML.State.Execute:
				_SeqStart := 0;
		ELSE
			_SeqStart := 0;
		END_CASE
		
END_CASE
END_METHOD

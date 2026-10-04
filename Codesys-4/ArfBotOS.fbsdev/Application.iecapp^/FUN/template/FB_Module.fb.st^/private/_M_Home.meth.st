METHOD PRIVATE _M_Home : BOOL

CASE _SeqHome OF
	0:
		//
		IF _Home THEN
			_Home := FALSE;
			M_Reset();
			_SeqHome := _SeqHome + 1;
		END_IF
	1:
		//
		CASE _UnitMode.CurrentState OF
			PACK_ML.State.Aborted:
				_UnitMode.Clear();
				
			PACK_ML.State.Stopped,
			PACK_ML.State.Complete:
				_UnitMode.Reset();
				_SeqHome := _SeqHome + 1;
				
			PACK_ML.State.Resetting,
			PACK_ML.State.Clearing,
			PACK_ML.State.Aborting:
				// just wait
		ELSE
			_UnitMode.Abort();
			
		END_CASE
		
	2:
		IF _UnitMode.CurrentState = PACK_ML.State.Idle THEN
			_SeqHome := 0;
		END_IF
		
END_CASE
END_METHOD

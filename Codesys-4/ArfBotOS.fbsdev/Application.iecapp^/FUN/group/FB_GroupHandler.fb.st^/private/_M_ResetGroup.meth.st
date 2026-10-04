METHOD PRIVATE _M_ResetGroup : BOOL
VAR_INPUT
END_VAR

CASE _SeqResetGroup OF
	0: // IDLE
		IF _ResetGroup THEN
			_ResetGroup := FALSE;
			//IF Error THEN
				_SeqResetGroup := _SeqResetGroup + 1;
			//END_IF
		END_IF
		
	1:
		Error := FALSE;
		ErrorID := SMC_NO_ERROR;
		_MC_GroupReset.Execute := TRUE;
		_SeqResetGroup := _SeqResetGroup + 1;
	
	2: 
		_MC_GroupReset.Execute := FALSE;
		//
		_SeqEnableGroup := 0;
		_SeqDisableGroup := 0;
		_SequenceSetKinematics := 0;
		_SequenceSetCoordTrans := 0;
		_SeqResetGroup := 0;
	
END_CASE
END_METHOD

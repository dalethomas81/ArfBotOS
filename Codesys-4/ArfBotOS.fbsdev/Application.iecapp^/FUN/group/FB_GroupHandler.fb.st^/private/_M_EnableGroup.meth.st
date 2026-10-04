METHOD PRIVATE _M_EnableGroup : BOOL
VAR_INPUT
END_VAR

CASE _SeqEnableGroup OF
	0: // IDLE
		IF _EnableGroup THEN
			_EnableGroup := FALSE;
			_GroupEnabled := FALSE;
			_MC_GroupEnable.Execute := FALSE;
			_SeqEnableGroup := _SeqEnableGroup + 1;
		END_IF
		
	1:
		_GroupPower := TRUE;
		_SeqEnableGroup := _SeqEnableGroup + 1;
		
	2:
		// TODO add timeout timer here
		IF _SMC_GroupPower.Error THEN
			Error := TRUE;
			ErrorID := _SMC_GroupPower.ErrorID;
			_SeqEnableGroup := 0;
		ELSIF _SMC_GroupPower.Status THEN
			_SeqEnableGroup := _SeqEnableGroup + 1;
		END_IF
		
	3:
		_MC_GroupEnable.Execute := TRUE;
		_SeqEnableGroup := _SeqEnableGroup + 1;
		
	4:
		// TODO add timeout timer here
		IF NOT _MC_GroupEnable.Busy THEN // this is a hack. we need to figure out is the group is already enabled first
			_GroupEnabled := TRUE;
			//_MC_GroupEnable.Execute := FALSE;
			_SeqEnableGroup := 0;
		END_IF;
END_CASE
END_METHOD

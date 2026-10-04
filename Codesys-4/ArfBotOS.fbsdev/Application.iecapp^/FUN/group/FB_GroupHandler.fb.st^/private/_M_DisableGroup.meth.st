METHOD PRIVATE _M_DisableGroup : BOOL
VAR_INPUT
END_VAR

CASE _SeqDisableGroup OF
	0: // IDLE
		IF _DisableGroup THEN
			_DisableGroup := FALSE;
			_MC_GroupDisable.Execute := FALSE;
			_MC_GroupEnable.Execute := FALSE;
			_SeqDisableGroup := _SeqDisableGroup + 1;
		END_IF
		
	1:
		_MC_GroupDisable.Execute := TRUE;
		_SeqDisableGroup := _SeqDisableGroup + 1;
		
	2:
		// TODO add timeout timer here
		IF NOT _MC_GroupDisable.Busy THEN // this is a hack. we need to figure out is the group is already disabled first
			_SeqDisableGroup := _SeqDisableGroup + 1;;
		END_IF
		
	3:
		_MC_GroupDisable.Execute := FALSE;
		_GroupEnabled := FALSE;
		_KinematicsSet := FALSE;
		_GroupPower := FALSE;
		_SeqDisableGroup := 0;
		
END_CASE
END_METHOD

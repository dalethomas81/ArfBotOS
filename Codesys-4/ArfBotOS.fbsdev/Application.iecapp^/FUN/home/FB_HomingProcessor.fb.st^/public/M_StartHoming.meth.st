METHOD PUBLIC M_StartHoming : BOOL
VAR_INPUT
END_VAR

IF _SequenceMain = STEP_IDLE THEN
	M_StartHoming := TRUE;
	_StartHoming := TRUE;
ELSE
	M_StartHoming := FALSE;
END_IF
END_METHOD

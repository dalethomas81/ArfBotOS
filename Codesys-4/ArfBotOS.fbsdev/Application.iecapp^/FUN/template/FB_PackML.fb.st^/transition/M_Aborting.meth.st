METHOD PROTECTED M_Aborting : BOOL
VAR_INPUT
END_VAR

_StateTimer.IN:=TRUE;

IF _StateTimer.Q THEN
	_StateTimer.IN:=FALSE;
	M_StateComplete();
END_IF
END_METHOD

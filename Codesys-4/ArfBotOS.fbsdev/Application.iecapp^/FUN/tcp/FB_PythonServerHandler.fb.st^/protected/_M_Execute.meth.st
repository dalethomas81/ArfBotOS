METHOD PROTECTED _M_Execute : BOOL
VAR_INPUT
	Command: STRING(255):='';
	Timeout: UDINT:=60;
END_VAR

_Execute := TRUE;
_Command:=Command;
_TimeoutTimer.PT := UDINT_TO_TIME(Timeout * 1000);
END_METHOD

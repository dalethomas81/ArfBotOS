METHOD PUBLIC M_EnableGroup : BOOL
VAR_INPUT
END_VAR

IF Enable AND NOT Error THEN
	M_EnableGroup := TRUE;
	_EnableGroup := TRUE;
	_GroupEnabled := FALSE;
ELSE
	M_EnableGroup := FALSE;
END_IF
END_METHOD

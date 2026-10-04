METHOD PUBLIC M_MoveToPose : BOOL
VAR_INPUT
	Pose: E_Pose;
END_VAR

IF Enabled 
	AND NOT _MoveCommandProcessor.Busy
	AND NOT _MoveCommandProcessor.Error THEN
		M_MoveToPose := TRUE;
		_MoveToPose[Pose] := TRUE;
ELSE
	M_MoveToPose := FALSE;
END_IF
END_METHOD

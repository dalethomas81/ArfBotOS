METHOD FB_Init : BOOL
VAR_INPUT
	bInitRetains : BOOL;
	bInCopyCode : BOOL;
END_VAR

IF NOT bInCopyCode THEN
	M_EnsureRegistered();
END_IF
FB_Init := TRUE;
END_METHOD

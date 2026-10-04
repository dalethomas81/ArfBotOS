METHOD FB_Exit : BOOL
VAR_INPUT
	bInCopyCode : BOOL;
END_VAR

IF NOT bInCopyCode THEN
	M_Unregister();
END_IF
FB_Exit := TRUE;
END_METHOD

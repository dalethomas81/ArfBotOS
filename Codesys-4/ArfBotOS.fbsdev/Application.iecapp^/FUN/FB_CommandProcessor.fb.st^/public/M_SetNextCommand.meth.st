METHOD PUBLIC M_SetNextCommand : BOOL // returns true if successful
VAR_INPUT
	CommandNumber: UINT;
END_VAR

IF CommandNumber <= GVL_App.PROG_CMDS_ARR_END AND CommandNumber >= GVL_App.PROG_CMDS_ARR_BEG THEN
	_CurrentCmdIdx := CommandNumber;
	M_SetNextCommand := TRUE;
ELSE
	M_SetNextCommand := FALSE;
END_IF
END_METHOD

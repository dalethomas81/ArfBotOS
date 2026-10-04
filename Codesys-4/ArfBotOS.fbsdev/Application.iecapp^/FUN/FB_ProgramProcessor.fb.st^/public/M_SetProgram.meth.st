METHOD PUBLIC M_SetProgram : BOOL
VAR_INPUT
	ProgramNumber: UINT;
END_VAR

IF ProgramNumber >= GVL_App.PROG_ARR_BEG AND ProgramNumber <= GVL_App.PROG_ARR_END THEN
	_CurrentPrgIdx := ProgramNumber;
	M_SetProgram := TRUE;
ELSE
	M_SetProgram := FALSE;
END_IF
END_METHOD

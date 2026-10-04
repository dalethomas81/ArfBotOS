METHOD PUBLIC M_CallProgram : bool
VAR_INPUT
	ProgramNumber: UINT; // program to call
	ReturnProgram: UINT; // number of the program to return to when the call program ends
	ReturnCommand: UINT; // number of the command to start from when called program ends
END_VAR

{warning disable C0371}
IF ProgramNumber <= GVL_App.PROG_ARR_END AND_THEN ProgramNumber >= GVL_App.PROG_ARR_BEG
	AND_THEN ReturnProgram <= GVL_App.PROG_ARR_END AND_THEN ReturnProgram >= GVL_App.PROG_ARR_BEG
	AND_THEN SUPER^.PropCurrentState = PACK_ML.State.Execute THEN
		IF _ProcessCommands.M_SetCommands(StartingCommand:=0, Commands:=Programs[ProgramNumber].Commands) THEN
			IF ReturnCommand <= GVL_App.PROG_CMDS_ARR_END + 1 AND_THEN ReturnCommand >= GVL_App.PROG_CMDS_ARR_BEG THEN // +1 because we want to be able to tell if calling program was on last command
				_Returning[ProgramNumber] := TRUE;
				_ReturnPrograms[ProgramNumber] := ReturnProgram;
				_ReturnCommands[ProgramNumber] := ReturnCommand;
			END_IF
			_CurrentPrgIdx := ProgramNumber;
			M_CallProgram := TRUE;
		ELSE
			M_CallProgram := FALSE;
		END_IF
ELSE
	M_CallProgram := FALSE;
END_IF
{warning restore C0371}
END_METHOD

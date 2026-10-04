METHOD PUBLIC M_SetCommands : BOOL
VAR_INPUT
	StartingCommand: UINT;
	Commands: ARRAY [GVL_App.PROG_CMDS_ARR_BEG..GVL_App.PROG_CMDS_ARR_END] OF DUT_Command;
END_VAR

IF StartingCommand >= GVL_App.PROG_CMDS_ARR_BEG AND StartingCommand <= GVL_App.PROG_CMDS_ARR_END + 1 THEN // +1 because we want to be able to tell if calling program was on last command
	_CurrentCmdIdx := StartingCommand;
	_Commands := Commands;
	M_SetCommands := TRUE;
ELSE
	M_SetCommands := FALSE;
END_IF
END_METHOD

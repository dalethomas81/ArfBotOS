FUNCTION_BLOCK FB_ParsePauseCmd
VAR_INPUT
	Command: STRING(GVL_App.COMMAND_LEN);
END_VAR
VAR_OUTPUT
	stCommand: ST_PauseCmd;
	Error: BOOL;
	ErrorDescription: STRING;
END_VAR
VAR
	_CmdParams: ARRAY [0..GVL_App.COMMAND_LEN] OF STRING;
END_VAR

// T#30s

Error:=FALSE;
ErrorDescription:='';

// split command string into commands
_CmdParams:=F_SplitCmd(Command:=Command);

stCommand.PauseTime := STRING_TO_TIME(_CmdParams[0]);
END_FUNCTION_BLOCK

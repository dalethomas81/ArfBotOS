FUNCTION F_BuildPauseCmd : STRING(GVL_App.COMMAND_LEN)
VAR_INPUT
	stCommand: ST_PauseCmd;
END_VAR
VAR
    _Command: STRING(GVL_App.COMMAND_LEN);
END_VAR
VAR CONSTANT
	_Delimiter: STRING:=' ';
END_VAR

// clear the command
_Command:='';

// build command
F_AddElement(_Command, _Delimiter, TIME_TO_STRING(stCommand.PauseTime));

// set function equal to command
F_BuildPauseCmd := _Command;
END_FUNCTION

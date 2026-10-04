FUNCTION_BLOCK FB_ParseOutputCmd
VAR_INPUT
	Command: STRING(GVL_App.COMMAND_LEN);
END_VAR
VAR_OUTPUT
	stCommand: ST_OutputCmd;
	Error: BOOL;
	ErrorDescription: STRING;
END_VAR
VAR
	_CmdParams: ARRAY [0..GVL_App.COMMAND_LEN] OF STRING;
	_OutputType: STRING;
END_VAR

// DIG +0001.00 +0001.00 (turn digital input 1 on)
// DIG +0001.00 +0000.00 (turn digital input 1 off)
// PWM +0001.00 +0030.00(set PWM 1 to position 30)

Error:=FALSE;
ErrorDescription:='';

// split command string into commands
_CmdParams:=F_SplitCmd(Command:=Command);

_OutputType := _CmdParams[0];
IF _OutputType = 'DIG' THEN
	stCommand.OutputType := DUT_OutputType.Digital;
ELSIF _OutputType = 'PWM' THEN
	stCommand.OutputType := DUT_OutputType.Pwm;
END_IF
stCommand.OutputTarget := STRING_TO_INT(_CmdParams[1]);
stCommand.OutputValue1 := STRING_TO_LREAL(_CmdParams[2]);
END_FUNCTION_BLOCK

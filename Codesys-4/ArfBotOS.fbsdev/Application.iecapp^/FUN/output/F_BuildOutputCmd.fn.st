FUNCTION F_BuildOutputCmd : STRING(GVL_App.COMMAND_LEN)
VAR_INPUT
	stCommand: ST_OutputCmd;
END_VAR
VAR
    _Command: STRING(GVL_App.COMMAND_LEN);
END_VAR
VAR CONSTANT
	_Delimiter: STRING:=' ';
	_OutTypeDig: STRING(3):='DIG';
	_OutTypePwm: STRING(3):='PWM';
END_VAR

// DIG +0001.00 +0001.00 (turn digital input 1 on)
// DIG +0001.00 +0000.00 (turn digital input 1 off)
// PWM +0001.00 +0030.00(set PWM 1 to position 30)

// clear the command
_Command:='';

//
CASE stCommand.OutputType OF
	DUT_OutputType.Digital:
		F_AddElement(_Command, _Delimiter, _OutTypeDig);
	DUT_OutputType.Pwm:
		F_AddElement(_Command, _Delimiter, _OutTypePwm);
END_CASE

//
F_AddElement(_Command, _Delimiter, INT_TO_STRING(stCommand.OutputTarget));

//
F_AddElement(_Command, _Delimiter, LREAL_TO_STRING(stCommand.OutputValue1));

// set function equal to command
F_BuildOutputCmd := _Command;
END_FUNCTION

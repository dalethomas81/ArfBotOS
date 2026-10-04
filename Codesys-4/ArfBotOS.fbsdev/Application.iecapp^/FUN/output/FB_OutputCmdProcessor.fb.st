FUNCTION_BLOCK FB_OutputCmdProcessor EXTENDS FB_Module
VAR_INPUT
	Enable: BOOL;
	Command: STRING(GVL_App.COMMAND_LEN):='';
END_VAR
VAR_IN_OUT
	Digital: ARRAY [GVL_App.DIG_OUT_ARR_BEG..GVL_App.DIG_OUT_ARR_END] OF BOOL;
	Pwm: ARRAY [GVL_App.PWM_OUT_ARR_BEG..GVL_App.PWM_OUT_ARR_END] OF BYTE;
END_VAR
VAR_OUTPUT
END_VAR
VAR
END_VAR
VAR
	_RunRtrig: R_TRIG;
	_SequenceDigital: INT;
	_SequencePwm: INT;
	
	_UnitModeCurrentState: PACK_ML.State;
	_OutputCommandParser: FB_ParseOutputCmd;
	_OutputType: DUT_OutputType;
	_OutputTarget: INT;
	_OutputValue1: LREAL;
END_VAR
VAR CONSTANT
	_STEP_DONE: 		INT := 1000;
	_STEP_IDLE: 		INT := 0000;
	_STEP_COMMAND: 		INT := 0010;
	_STEP_TWO: 			INT := 0020;
	_STEP_ERR: 			INT := 0030;
END_VAR

SUPER^();

// PackML state machine implementation
_UnitModeCurrentState := SUPER^._UnitMode.CurrentState;

//
CASE SUPER^._SeqMain OF
	
    _STEP_DONE:
		SUPER^._SeqMain := _STEP_IDLE;
        
    _STEP_IDLE:
        // hang out here untill called to run
		
    _STEP_COMMAND:
		_OutputCommandParser(Command:=Command);
		IF _OutputCommandParser.Error THEN
			SUPER^.M_Error(Description:=_OutputCommandParser.ErrorDescription);
			SUPER^._SeqMain := _STEP_ERR;
		ELSE
			_OutputType := _OutputCommandParser.stCommand.OutputType;
			_OutputTarget := _OutputCommandParser.stCommand.OutputTarget;
			_OutputValue1 := _OutputCommandParser.stCommand.OutputValue1;
			
			CASE _OutputType OF
				DUT_OutputType.Digital: //
					_SequenceDigital := 1;
					SUPER^._SeqMain := _STEP_TWO;
				DUT_OutputType.Pwm: //
					_SequencePwm := 1;
					SUPER^._SeqMain := _STEP_TWO;
			ELSE
				SUPER^._SeqMain := _STEP_DONE;
				
			END_CASE
		END_IF
		
	_STEP_TWO:
		IF _SequenceDigital = 0 
			AND _SequencePwm = 0 THEN
				// next, please
				SUPER^._SeqMain := _STEP_DONE;
		END_IF;

END_CASE

CASE _SequenceDigital OF
	0: //
	1: //
		THIS^.Digital[_OutputTarget] := _OutputValue1 > 0;
		_SequenceDigital := 0;
END_CASE

CASE _SequencePwm OF
	0: //
	1: //
		THIS^.Pwm[_OutputTarget] := LREAL_TO_BYTE(_OutputValue1);
		_SequencePwm := 0;
	
END_CASE
END_FUNCTION_BLOCK

FUNCTION_BLOCK FB_PauseCmdProcessor EXTENDS FB_Module
VAR_INPUT
	Command: STRING(GVL_App.COMMAND_LEN):='';
END_VAR
VAR_OUTPUT
	ElapsedTime: TIME;
	RemainingTime: TIME;
END_VAR
VAR
	_Timer: TON;
	_Timer_IN: BOOL;
	_Timer_PT: TIME;
	_Timer_Q: BOOL;
	_Timer_ET: TIME;
END_VAR
VAR
	_UnitModeCurrentState: PACK_ML.State;
	_ParsePauseCmd: FB_ParsePauseCmd;
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
 		_Timer_IN := FALSE;
		SUPER^._SeqMain := _STEP_IDLE;
        
    _STEP_IDLE:
		// hang out here until called to run
        
    _STEP_COMMAND:
		_ParsePauseCmd(Command:=Command);
		_Timer_PT := _ParsePauseCmd.stCommand.PauseTime;
 		_Timer_IN := TRUE;
		// next, please
		SUPER^._SeqMain := _STEP_TWO;
		
	_STEP_TWO:
		IF _Timer.Q THEN
			// next, please
			SUPER^._SeqMain := _STEP_DONE;
		END_IF

END_CASE

_Timer(
	IN:= _Timer_IN, 
	PT:= _Timer_PT, 
	Q=> _Timer_Q, 
	ET=> _Timer_ET);
	
ElapsedTime := _Timer_ET;
RemainingTime := _Timer_PT - _Timer_ET;
END_FUNCTION_BLOCK

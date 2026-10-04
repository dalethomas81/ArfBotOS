FUNCTION F_ChangeJumpRow : BOOL
VAR_IN_OUT
	Command: DUT_Command;
END_VAR
VAR_INPUT
	BeginningRow: DINT;
	Change: INT;
END_VAR
VAR
	_LogicCmdParser : FB_ParseLogicCmd;
	_stLogicCmd : ST_LogicCmd;
END_VAR

// change the row that the jump function will jump to by the amount of the change
F_ChangeJumpRow := FALSE;
IF Command.CommandType = DUT_CommandType.LogicCommand THEN
	// parse the command string
	_LogicCmdParser(Command:=Command.CommandString, stCommand=>_stLogicCmd);
	
	// check the THEN function
	IF _stLogicCmd.THEN_Function = E_LogicFunction.JUMP AND_THEN _stLogicCmd.THEN_Function_Idx >= BeginningRow THEN
		_stLogicCmd.THEN_Function_Idx := _stLogicCmd.THEN_Function_Idx + Change;
		F_ChangeJumpRow := TRUE;
	END_IF
	
	// check the ELSE function
	IF _stLogicCmd.ELSE_Function = E_LogicFunction.JUMP AND_THEN _stLogicCmd.ELSE_Function_Idx >= BeginningRow  THEN
		_stLogicCmd.ELSE_Function_Idx := _stLogicCmd.ELSE_Function_Idx + Change;
		F_ChangeJumpRow := TRUE;
	END_IF
	
	// set the command
	IF F_ChangeJumpRow THEN
		Command.CommandString := F_BuildLogicCmd(stCommand:=_stLogicCmd);
	END_IF
	
ELSE
	F_ChangeJumpRow := FALSE;
	
END_IF
END_FUNCTION

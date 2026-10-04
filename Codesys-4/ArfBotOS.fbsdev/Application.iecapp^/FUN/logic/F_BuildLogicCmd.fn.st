FUNCTION F_BuildLogicCmd : STRING(GVL_App.COMMAND_LEN)
VAR_INPUT
	stCommand: ST_LogicCmd;
END_VAR
VAR
    _Command: STRING(GVL_App.COMMAND_LEN);
END_VAR
VAR CONSTANT
	_Delimiter: STRING:=' ';
END_VAR

// IF PASS|INPUT|REGISTER|COUNTER <index> >|<|=|>=|<= <value>
// THEN OUTPUT|JUMP|CNTUP|CNTDN|CNTRST|REGISTER|NOP <index> <value>
// ELSE OUTPUT|JUMP|CNTUP|CNTDN|CNTRST|REGISTER|NOP <index> <value>
//
// IF_Condition
// IF_Condition_Idx
// IF_Condition_Operator
// IF_Condition_Operator_Compare
// THEN_Function
// THEN_Function_Value
// THEN_Function_Idx
// ELSE_Function
// ELSE_Function_Idx
// ELSE_Function_Value

// clear the command
_Command:='';

// build command
F_AddElement(_Command, _Delimiter, 'IF');
F_AddElement(_Command, _Delimiter, TO_STRING(stCommand.IF_Condition));
IF stCommand.IF_Condition <> E_LogicIfCondition.PASS THEN
	F_AddElement(_Command, _Delimiter, INT_TO_STRING(stCommand.IF_Condition_Idx));
	F_AddElement(_Command, _Delimiter, TO_STRING(stCommand.IF_Condition_Operator));
	F_AddElement(_Command, _Delimiter, UDINT_TO_STRING(stCommand.IF_Condition_Operator_Value));
END_IF
F_AddElement(_Command, _Delimiter, 'THEN');
F_AddElement(_Command, _Delimiter, TO_STRING(stCommand.THEN_Function));
IF stCommand.THEN_Function <> E_LogicFunction.NOP THEN
	F_AddElement(_Command, _Delimiter, INT_TO_STRING(stCommand.THEN_Function_Idx));
	IF stCommand.THEN_Function = E_LogicFunction.OUTPUT OR 
		stCommand.THEN_Function = E_LogicFunction.REGISTER OR 
		stCommand.THEN_Function = E_LogicFunction.FLAG THEN
			F_AddElement(_Command, _Delimiter, UDINT_TO_STRING(stCommand.THEN_Function_Value));
	END_IF
END_IF
IF stCommand.IF_Condition <> E_LogicIfCondition.PASS THEN
	F_AddElement(_Command, _Delimiter, 'ELSE');
	F_AddElement(_Command, _Delimiter, TO_STRING(stCommand.ELSE_Function));
	IF stCommand.ELSE_Function <> E_LogicFunction.NOP THEN
		F_AddElement(_Command, _Delimiter, INT_TO_STRING(stCommand.ELSE_Function_Idx));
		IF stCommand.ELSE_Function = E_LogicFunction.OUTPUT OR
			stCommand.ELSE_Function = E_LogicFunction.REGISTER OR
			stCommand.ELSE_Function = E_LogicFunction.FLAG THEN
				F_AddElement(_Command, _Delimiter, UDINT_TO_STRING(stCommand.ELSE_Function_Value));
		END_IF
	END_IF
END_IF

// set function equal to command
F_BuildLogicCmd := _Command;
END_FUNCTION

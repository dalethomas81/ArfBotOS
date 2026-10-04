FUNCTION_BLOCK FB_ParseLogicCmd
VAR_INPUT
	Command: STRING(GVL_App.COMMAND_LEN);
END_VAR
VAR_OUTPUT
	stCommand: ST_LogicCmd;
	Error: BOOL;
	ErrorDescription: STRING;
END_VAR
VAR
	_CmdParams: ARRAY [0..GVL_App.COMMAND_LEN] OF STRING;
	_i, _j, _k: INT;
END_VAR

// IF PASS|INPUT|REGISTER|COUNTER <index> >|<|=|>=|<= <value>
// THEN OUTPUT|JUMP|CALL|CNTUP|CNTDN|CNTRST|REGISTER|NOP <index> <value>
// ELSE OUTPUT|JUMP|CALL|CNTUP|CNTDN|CNTRST|REGISTER|NOP <index> <value>
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

Error:=FALSE;
ErrorDescription:='';

// split command string into commands
_CmdParams:=F_SplitCmd(Command:=Command);

//
_i := 0;

// disregard the 'IF'
_i := _i + 1;

//
IF _CmdParams[_i] = TO_STRING(E_LogicIfCondition.PASS) THEN
	stCommand.IF_Condition := E_LogicIfCondition.PASS;
ELSIF  _CmdParams[_i] = TO_STRING(E_LogicIfCondition.INPUT) THEN
	stCommand.IF_Condition := E_LogicIfCondition.INPUT;
ELSIF  _CmdParams[_i] = TO_STRING(E_LogicIfCondition.REGISTER) THEN
	stCommand.IF_Condition := E_LogicIfCondition.REGISTER;
ELSIF  _CmdParams[_i] = TO_STRING(E_LogicIfCondition.COUNTER) THEN
	stCommand.IF_Condition := E_LogicIfCondition.COUNTER;
ELSIF  _CmdParams[_i] = TO_STRING(E_LogicIfCondition.FLAG) THEN
	stCommand.IF_Condition := E_LogicIfCondition.FLAG;
END_IF
_i := _i + 1;

//
IF stCommand.IF_Condition <> E_LogicIfCondition.PASS THEN
	
	//
	stCommand.IF_Condition_Idx := STRING_TO_INT(_CmdParams[_i]);
	_i := _i + 1;
	
	//
	IF _CmdParams[_i] = TO_STRING(E_LogicIfConditionOperator.GRT) THEN
		stCommand.IF_Condition_Operator := E_LogicIfConditionOperator.GRT;
	ELSIF _CmdParams[_i] = TO_STRING(E_LogicIfConditionOperator.LES) THEN
		stCommand.IF_Condition_Operator := E_LogicIfConditionOperator.LES;                   
	ELSIF _CmdParams[_i] = TO_STRING(E_LogicIfConditionOperator.EQU) THEN
		stCommand.IF_Condition_Operator := E_LogicIfConditionOperator.EQU;
	ELSIF _CmdParams[_i] = TO_STRING(E_LogicIfConditionOperator.GRE) THEN
		stCommand.IF_Condition_Operator := E_LogicIfConditionOperator.GRE;
	ELSIF _CmdParams[_i] = TO_STRING(E_LogicIfConditionOperator.LEQ) THEN
		stCommand.IF_Condition_Operator := E_LogicIfConditionOperator.LEQ;
	END_IF
	_i := _i + 1;
	
	//
	stCommand.IF_Condition_Operator_Value := STRING_TO_UDINT(_CmdParams[_i]);
	_i := _i + 1;
	
END_IF

// disregard the 'THEN'
_i := _i + 1;

//
IF _CmdParams[_i] = TO_STRING(E_LogicFunction.OUTPUT) THEN
	stCommand.THEN_Function := E_LogicFunction.OUTPUT;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.JUMP) THEN
	stCommand.THEN_Function := E_LogicFunction.JUMP;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.CALL) THEN
	stCommand.THEN_Function := E_LogicFunction.CALL;           
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.CNTUP) THEN
	stCommand.THEN_Function := E_LogicFunction.CNTUP;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.CNTDN) THEN
	stCommand.THEN_Function := E_LogicFunction.CNTDN;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.CNTRST) THEN
	stCommand.THEN_Function := E_LogicFunction.CNTRST;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.REGISTER) THEN
	stCommand.THEN_Function := E_LogicFunction.REGISTER;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.FLAG) THEN
	stCommand.THEN_Function := E_LogicFunction.FLAG;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.NOP) THEN
	stCommand.THEN_Function := E_LogicFunction.NOP;
END_IF
_i := _i + 1;

//
IF stCommand.THEN_Function <> E_LogicFunction.NOP THEN
	stCommand.THEN_Function_Idx := STRING_TO_INT(_CmdParams[_i]);
	_i := _i + 1;
	IF stCommand.THEN_Function = E_LogicFunction.OUTPUT OR
		stCommand.THEN_Function = E_LogicFunction.REGISTER OR
		stCommand.THEN_Function = E_LogicFunction.FLAG THEN
			stCommand.THEN_Function_Value := STRING_TO_UDINT(_CmdParams[_i]);
			_i := _i + 1;
	END_IF
END_IF

// disregard the 'ELSE'
_i := _i + 1;

//
IF _CmdParams[_i] = TO_STRING(E_LogicFunction.OUTPUT) THEN
	stCommand.ELSE_Function := E_LogicFunction.OUTPUT;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.JUMP) THEN
	stCommand.ELSE_Function := E_LogicFunction.JUMP;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.CALL) THEN
	stCommand.ELSE_Function := E_LogicFunction.CALL;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.CNTUP) THEN
	stCommand.ELSE_Function := E_LogicFunction.CNTUP;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.CNTDN) THEN
	stCommand.ELSE_Function := E_LogicFunction.CNTDN;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.CNTRST) THEN
	stCommand.ELSE_Function := E_LogicFunction.CNTRST;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.REGISTER) THEN
	stCommand.ELSE_Function := E_LogicFunction.REGISTER;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.FLAG) THEN
	stCommand.ELSE_Function := E_LogicFunction.FLAG;
ELSIF _CmdParams[_i] = TO_STRING(E_LogicFunction.NOP) THEN
	stCommand.ELSE_Function := E_LogicFunction.NOP;
END_IF
_i := _i + 1;

//
IF stCommand.ELSE_Function <> E_LogicFunction.NOP THEN
	stCommand.ELSE_Function_Idx := STRING_TO_INT(_CmdParams[_i]);
	_i := _i + 1;
	IF stCommand.ELSE_Function = E_LogicFunction.OUTPUT OR
		stCommand.ELSE_Function = E_LogicFunction.REGISTER OR
		stCommand.ELSE_Function = E_LogicFunction.FLAG THEN
			stCommand.ELSE_Function_Value := STRING_TO_UDINT(_CmdParams[_i]);
			_i := _i + 1;
	END_IF
END_IF
END_FUNCTION_BLOCK

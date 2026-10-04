FUNCTION F_VarSetCmdOperate : LREAL
VAR_INPUT
	Operand1: LREAL;
	Operation: DUT_VarSetCmd_OperType;
	Operand2: LREAL;
END_VAR
VAR
	_epsilon : REAL := 0.00000001;
END_VAR

CASE Operation OF
	DUT_VarSetCmd_OperType.NOP:
		F_VarSetCmdOperate := Operand1;
		
	DUT_VarSetCmd_OperType.EQUALS:
		F_VarSetCmdOperate := Operand2;
		
	DUT_VarSetCmd_OperType.PLUS:
		F_VarSetCmdOperate := Operand1 + Operand2;
		
	DUT_VarSetCmd_OperType.MINUS:
		F_VarSetCmdOperate := Operand1 - Operand2;
		
	DUT_VarSetCmd_OperType.TIMES:
		F_VarSetCmdOperate := Operand1 * Operand2;
		
	DUT_VarSetCmd_OperType.DIVIDED_BY:
		IF Operand2 <> 0 THEN
			F_VarSetCmdOperate := Operand1 / Operand2;
		ELSE
			F_VarSetCmdOperate := Operand1 / _epsilon;
		END_IF
		
END_CASE
END_FUNCTION

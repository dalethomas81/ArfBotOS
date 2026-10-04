FUNCTION_BLOCK FB_ParseVarSetCmd
VAR_INPUT
	Command: STRING(GVL_App.COMMAND_LEN);
END_VAR
VAR_OUTPUT
	stCommand: ST_VarSetCmd;
	Error: BOOL;
	ErrorDescription: STRING;
END_VAR
VAR
	_i: INT;
	_CmdParams: ARRAY [0..GVL_App.COMMAND_LEN] OF STRING;
END_VAR
VAR CONSTANT
	_Delimiter: STRING:=' ';
	_ArrayTypeS: STRING(3):='STO';
	_ArrayTypeV: STRING(3):='VIS';
	_ArrayTypeF: STRING(3):='FLG';
	_OperationTypeN: STRING(1):='N';
	_OperationTypeE: STRING(1):='E';
	_OperationTypeA: STRING(1):='A';
	_OperationTypeS: STRING(1):='S';
	_OperationTypeM: STRING(1):='M';
	_OperationTypeD: STRING(1):='D';
END_VAR

// S|V|F index E|A|S|M|D x E|A|S|M|D yE|A|S|M|D z E|A|S|M|D a E|A|S|M|D b E|A|S|M|D c
// S 0000 E|A|S|M|D +0000.00 E|A|S|M|D +0000.00 E|A|S|M|D +0000.00 E|A|S|M|D +0000.00 E|A|S|M|D +0000.00 E|A|S|M|D +0000.00 
// V 0000 E|A|S|M|D +0000.00 E|A|S|M|D +0000.00 E|A|S|M|D +0000.00 E|A|S|M|D +0000.00 E|A|S|M|D +0000.00 E|A|S|M|D +0000.00
// F 0000 1

Error:=FALSE;
ErrorDescription:='';

// split command string into commands
_CmdParams:=F_SplitCmd(Command:=Command, _Delimiter);

//
_i:=0;
IF _CmdParams[_i] = _ArrayTypeS  THEN
	stCommand.ArrayType := DUT_Command_ArrayType.StoredPosition;
ELSIF _CmdParams[_i] = _ArrayTypeV THEN
	stCommand.ArrayType := DUT_Command_ArrayType.VisionRegister;
ELSIF _CmdParams[_i] = _ArrayTypeF THEN
	stCommand.ArrayType := DUT_Command_ArrayType.Flag;
ELSE
	Error:=TRUE;
	ErrorDescription:='Incorrect array type';
END_IF

//
_i := _i + 1;
stCommand.ArrayIndex := STRING_TO_INT(_CmdParams[_i]);
IF ((stCommand.ArrayIndex < GVL_App.POS_ARR_BEG OR stCommand.ArrayIndex > GVL_App.POS_ARR_END) AND stCommand.ArrayType = DUT_Command_ArrayType.StoredPosition)
	OR ((stCommand.ArrayIndex < GVL_App.VIS_ARR_BEG OR stCommand.ArrayIndex > GVL_App.VIS_ARR_END) AND stCommand.ArrayType = DUT_Command_ArrayType.VisionRegister)
	OR ((stCommand.ArrayIndex < GVL_App.FLG_ARR_BEG OR stCommand.ArrayIndex > GVL_App.FLG_ARR_END) AND stCommand.ArrayType = DUT_Command_ArrayType.Flag) THEN
		Error:=TRUE;
		ErrorDescription:='Incorrect array type';
END_IF

//
_i := _i + 1;
IF _CmdParams[_i] = _OperationTypeN THEN
	stCommand.XOperType := DUT_VarSetCmd_OperType.NOP;
ELSIF _CmdParams[_i] = _OperationTypeE THEN
	stCommand.XOperType := DUT_VarSetCmd_OperType.EQUALS;
ELSIF _CmdParams[_i] = _OperationTypeA THEN
	stCommand.XOperType := DUT_VarSetCmd_OperType.PLUS;
ELSIF _CmdParams[_i] = _OperationTypeS THEN
	stCommand.XOperType := DUT_VarSetCmd_OperType.MINUS;
ELSIF _CmdParams[_i] = _OperationTypeM THEN
	stCommand.XOperType := DUT_VarSetCmd_OperType.TIMES;
ELSIF _CmdParams[_i] = _OperationTypeD THEN
	stCommand.XOperType := DUT_VarSetCmd_OperType.DIVIDED_BY;
END_IF
_i := _i + 1;
stCommand.Coordinates.X := STRING_TO_LREAL(_CmdParams[_i]);

_i := _i + 1;
IF _CmdParams[_i] = _OperationTypeN THEN
	stCommand.YOperType := DUT_VarSetCmd_OperType.NOP;
ELSIF _CmdParams[_i] = _OperationTypeE THEN
	stCommand.YOperType := DUT_VarSetCmd_OperType.EQUALS;
ELSIF _CmdParams[_i] = _OperationTypeA THEN
	stCommand.YOperType := DUT_VarSetCmd_OperType.PLUS;
ELSIF _CmdParams[_i] = _OperationTypeS THEN
	stCommand.YOperType := DUT_VarSetCmd_OperType.MINUS;
ELSIF _CmdParams[_i] = _OperationTypeM THEN
	stCommand.YOperType := DUT_VarSetCmd_OperType.TIMES;
ELSIF _CmdParams[_i] = _OperationTypeD THEN
	stCommand.YOperType := DUT_VarSetCmd_OperType.DIVIDED_BY;
END_IF
_i := _i + 1;
stCommand.Coordinates.Y := STRING_TO_LREAL(_CmdParams[_i]);

_i := _i + 1;
IF _CmdParams[_i] = _OperationTypeN THEN
	stCommand.ZOperType := DUT_VarSetCmd_OperType.NOP;
ELSIF _CmdParams[_i] = _OperationTypeE THEN
	stCommand.ZOperType := DUT_VarSetCmd_OperType.EQUALS;
ELSIF _CmdParams[_i] = _OperationTypeA THEN
	stCommand.ZOperType := DUT_VarSetCmd_OperType.PLUS;
ELSIF _CmdParams[_i] = _OperationTypeS THEN
	stCommand.ZOperType := DUT_VarSetCmd_OperType.MINUS;
ELSIF _CmdParams[_i] = _OperationTypeM THEN
	stCommand.ZOperType := DUT_VarSetCmd_OperType.TIMES;
ELSIF _CmdParams[_i] = _OperationTypeD THEN
	stCommand.ZOperType := DUT_VarSetCmd_OperType.DIVIDED_BY;
END_IF
_i := _i + 1;
stCommand.Coordinates.Z := STRING_TO_LREAL(_CmdParams[_i]);

_i := _i + 1;
IF _CmdParams[_i] = _OperationTypeN THEN
	stCommand.AOperType := DUT_VarSetCmd_OperType.NOP;
ELSIF _CmdParams[_i] = _OperationTypeE THEN
	stCommand.AOperType := DUT_VarSetCmd_OperType.EQUALS;
ELSIF _CmdParams[_i] = _OperationTypeA THEN
	stCommand.AOperType := DUT_VarSetCmd_OperType.PLUS;
ELSIF _CmdParams[_i] = _OperationTypeS THEN
	stCommand.AOperType := DUT_VarSetCmd_OperType.MINUS;
ELSIF _CmdParams[_i] = _OperationTypeM THEN
	stCommand.AOperType := DUT_VarSetCmd_OperType.TIMES;
ELSIF _CmdParams[_i] = _OperationTypeD THEN
	stCommand.AOperType := DUT_VarSetCmd_OperType.DIVIDED_BY;
END_IF
_i := _i + 1;
stCommand.Coordinates.A := STRING_TO_LREAL(_CmdParams[_i]);

_i := _i + 1;
IF _CmdParams[_i] = _OperationTypeN THEN
	stCommand.BOperType := DUT_VarSetCmd_OperType.NOP;
ELSIF _CmdParams[_i] = _OperationTypeE THEN
	stCommand.BOperType := DUT_VarSetCmd_OperType.EQUALS;
ELSIF _CmdParams[_i] = _OperationTypeA THEN
	stCommand.BOperType := DUT_VarSetCmd_OperType.PLUS;
ELSIF _CmdParams[_i] = _OperationTypeS THEN
	stCommand.BOperType := DUT_VarSetCmd_OperType.MINUS;
ELSIF _CmdParams[_i] = _OperationTypeM THEN
	stCommand.BOperType := DUT_VarSetCmd_OperType.TIMES;
ELSIF _CmdParams[_i] = _OperationTypeD THEN
	stCommand.BOperType := DUT_VarSetCmd_OperType.DIVIDED_BY;
END_IF
_i := _i + 1;
stCommand.Coordinates.B := STRING_TO_LREAL(_CmdParams[_i]);

_i := _i + 1;
IF _CmdParams[_i] = _OperationTypeN THEN
	stCommand.COperType := DUT_VarSetCmd_OperType.NOP;
ELSIF _CmdParams[_i] = _OperationTypeE THEN
	stCommand.COperType := DUT_VarSetCmd_OperType.EQUALS;
ELSIF _CmdParams[_i] = _OperationTypeA THEN
	stCommand.COperType := DUT_VarSetCmd_OperType.PLUS;
ELSIF _CmdParams[_i] = _OperationTypeS THEN
	stCommand.COperType := DUT_VarSetCmd_OperType.MINUS;
ELSIF _CmdParams[_i] = _OperationTypeM THEN
	stCommand.COperType := DUT_VarSetCmd_OperType.TIMES;
ELSIF _CmdParams[_i] = _OperationTypeD THEN
	stCommand.COperType := DUT_VarSetCmd_OperType.DIVIDED_BY;
END_IF
_i := _i + 1;
stCommand.Coordinates.C := STRING_TO_LREAL(_CmdParams[_i]);
END_FUNCTION_BLOCK

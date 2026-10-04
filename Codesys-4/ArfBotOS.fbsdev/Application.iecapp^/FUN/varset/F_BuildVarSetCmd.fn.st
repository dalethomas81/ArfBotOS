FUNCTION F_BuildVarSetCmd : STRING(GVL_App.COMMAND_LEN)
VAR_INPUT
	stCommand: ST_VarSetCmd;
END_VAR
VAR
    _Command: STRING(GVL_App.COMMAND_LEN);
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

// clear the command
_Command:='';

// build the command
CASE stCommand.ArrayType OF
	DUT_Command_ArrayType.StoredPosition:
		F_AddElement(_Command, _Delimiter, _ArrayTypeS);
	DUT_Command_ArrayType.VisionRegister:
		F_AddElement(_Command, _Delimiter, _ArrayTypeV);
	DUT_Command_ArrayType.Flag:
		F_AddElement(_Command, _Delimiter, _ArrayTypeF);
END_CASE

// add the array index
F_AddElement(_Command, _Delimiter, INT_TO_STRING(stCommand.ArrayIndex));

// add the coordinates
IF stCommand.ArrayType = DUT_Command_ArrayType.StoredPosition 
	OR stCommand.ArrayType = DUT_Command_ArrayType.VisionRegister THEN
	
		CASE stCommand.XOperType OF
			DUT_VarSetCmd_OperType.NOP:
				F_AddElement(_Command, _Delimiter, _OperationTypeN);
			DUT_VarSetCmd_OperType.EQUALS:
				F_AddElement(_Command, _Delimiter, _OperationTypeE);
			DUT_VarSetCmd_OperType.PLUS:
				F_AddElement(_Command, _Delimiter, _OperationTypeA);
			DUT_VarSetCmd_OperType.MINUS:
				F_AddElement(_Command, _Delimiter, _OperationTypeS);
			DUT_VarSetCmd_OperType.TIMES:
				F_AddElement(_Command, _Delimiter, _OperationTypeM);
			DUT_VarSetCmd_OperType.DIVIDED_BY:
				F_AddElement(_Command, _Delimiter, _OperationTypeD);
		END_CASE
		F_AddElement(_Command, _Delimiter, LREAL_TO_STRING(stCommand.Coordinates.X));
		
		CASE stCommand.YOperType OF
			DUT_VarSetCmd_OperType.NOP:
				F_AddElement(_Command, _Delimiter, _OperationTypeN);
			DUT_VarSetCmd_OperType.EQUALS:
				F_AddElement(_Command, _Delimiter, _OperationTypeE);
			DUT_VarSetCmd_OperType.PLUS:
				F_AddElement(_Command, _Delimiter, _OperationTypeA);
			DUT_VarSetCmd_OperType.MINUS:
				F_AddElement(_Command, _Delimiter, _OperationTypeS);
			DUT_VarSetCmd_OperType.TIMES:
				F_AddElement(_Command, _Delimiter, _OperationTypeM);
			DUT_VarSetCmd_OperType.DIVIDED_BY:
				F_AddElement(_Command, _Delimiter, _OperationTypeD);
		END_CASE
		F_AddElement(_Command, _Delimiter, LREAL_TO_STRING(stCommand.Coordinates.Y));
		
		CASE stCommand.ZOperType OF
			DUT_VarSetCmd_OperType.NOP:
				F_AddElement(_Command, _Delimiter, _OperationTypeN);
			DUT_VarSetCmd_OperType.EQUALS:
				F_AddElement(_Command, _Delimiter, _OperationTypeE);
			DUT_VarSetCmd_OperType.PLUS:
				F_AddElement(_Command, _Delimiter, _OperationTypeA);
			DUT_VarSetCmd_OperType.MINUS:
				F_AddElement(_Command, _Delimiter, _OperationTypeS);
			DUT_VarSetCmd_OperType.TIMES:
				F_AddElement(_Command, _Delimiter, _OperationTypeM);
			DUT_VarSetCmd_OperType.DIVIDED_BY:
				F_AddElement(_Command, _Delimiter, _OperationTypeD);
		END_CASE;
		F_AddElement(_Command, _Delimiter, LREAL_TO_STRING(stCommand.Coordinates.Z));
		
		CASE stCommand.AOperType OF
			DUT_VarSetCmd_OperType.NOP:
				F_AddElement(_Command, _Delimiter, _OperationTypeN);
			DUT_VarSetCmd_OperType.EQUALS:
				F_AddElement(_Command, _Delimiter, _OperationTypeE);
			DUT_VarSetCmd_OperType.PLUS:
				F_AddElement(_Command, _Delimiter, _OperationTypeA);
			DUT_VarSetCmd_OperType.MINUS:
				F_AddElement(_Command, _Delimiter, _OperationTypeS);
			DUT_VarSetCmd_OperType.TIMES:
				F_AddElement(_Command, _Delimiter, _OperationTypeM);
			DUT_VarSetCmd_OperType.DIVIDED_BY:
				F_AddElement(_Command, _Delimiter, _OperationTypeD);
		END_CASE
		F_AddElement(_Command, _Delimiter, LREAL_TO_STRING(stCommand.Coordinates.A));
		
		CASE stCommand.BOperType OF
			DUT_VarSetCmd_OperType.NOP:
				F_AddElement(_Command, _Delimiter, _OperationTypeN);
			DUT_VarSetCmd_OperType.EQUALS:
				F_AddElement(_Command, _Delimiter, _OperationTypeE);
			DUT_VarSetCmd_OperType.PLUS:
				F_AddElement(_Command, _Delimiter, _OperationTypeA);
			DUT_VarSetCmd_OperType.MINUS:
				F_AddElement(_Command, _Delimiter, _OperationTypeS);
			DUT_VarSetCmd_OperType.TIMES:
				F_AddElement(_Command, _Delimiter, _OperationTypeM);
			DUT_VarSetCmd_OperType.DIVIDED_BY:
				F_AddElement(_Command, _Delimiter, _OperationTypeD);
		END_CASE
		F_AddElement(_Command, _Delimiter, LREAL_TO_STRING(stCommand.Coordinates.B));
		
		CASE stCommand.COperType OF
			DUT_VarSetCmd_OperType.NOP:
				F_AddElement(_Command, _Delimiter, _OperationTypeN);
			DUT_VarSetCmd_OperType.EQUALS:
				F_AddElement(_Command, _Delimiter, _OperationTypeE);
			DUT_VarSetCmd_OperType.PLUS:
				F_AddElement(_Command, _Delimiter, _OperationTypeA);
			DUT_VarSetCmd_OperType.MINUS:
				F_AddElement(_Command, _Delimiter, _OperationTypeS);
			DUT_VarSetCmd_OperType.TIMES:
				F_AddElement(_Command, _Delimiter, _OperationTypeM);
			DUT_VarSetCmd_OperType.DIVIDED_BY:
				F_AddElement(_Command, _Delimiter, _OperationTypeD);
		END_CASE
		F_AddElement(_Command, _Delimiter, LREAL_TO_STRING(stCommand.Coordinates.C));
		
ELSIF stCommand.ArrayType = DUT_Command_ArrayType.Flag THEN
		// we only send the x for readability. it will work if we send them all.
		F_AddElement(_Command, _Delimiter, LREAL_TO_STRING(stCommand.Coordinates.X));
		
END_IF

// set function equal to command
F_BuildVarSetCmd := _Command;
END_FUNCTION

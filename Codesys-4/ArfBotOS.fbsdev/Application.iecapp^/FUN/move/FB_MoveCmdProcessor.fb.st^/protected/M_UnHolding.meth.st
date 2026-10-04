METHOD PROTECTED M_Unholding : BOOL

// here the operator called unhold (most likely from M_Start)
_MC_GroupHalt.Execute := FALSE;
CASE _MoveCommandParser.stCommand.MoveType OF 
	DUT_Command_MoveType.DirectMove:
		IF _MoveCommandParser.stCommand.PositionType = DUT_Command_Positiontype.AbsolutePosition THEN
			_MC_MoveDirectAbsolute.Execute := TRUE;
		ELSE
			_MC_MoveDirectRelative.Execute := TRUE;
		END_IF
	DUT_Command_MoveType.LinearMove:
		IF _MoveCommandParser.stCommand.PositionType = DUT_Command_Positiontype.AbsolutePosition THEN
			_MC_MoveLinearAbsolute.Execute := TRUE;
		ELSE
			_MC_MoveLinearRelative.Execute := TRUE;
		END_IF
	DUT_Command_MoveType.CircularMove:
		IF _MoveCommandParser.stCommand.PositionType = DUT_Command_Positiontype.AbsolutePosition THEN
			_MC_MoveCircularAbsolute.Execute := TRUE;
		ELSE
			_MC_MoveCircularRelative.Execute := TRUE;
		END_IF
END_CASE
//
SUPER^.M_StateComplete();
END_METHOD

METHOD PROTECTED M_Holding : BOOL

CASE SUPER^._SeqHolding OF
	0:
		_MC_MoveCircularAbsolute.Execute := FALSE;
		_MC_MoveCircularRelative.Execute := FALSE;
		_MC_MoveDirectAbsolute.Execute := FALSE;
		_MC_MoveDirectRelative.Execute := FALSE;
		_MC_MoveLinearAbsolute.Execute := FALSE;
		_MC_MoveLinearRelative.Execute := FALSE;
		_MC_GroupHalt.Execute:=TRUE;
		_SeqHolding := _SeqHolding + 1;
	1:
		IF _MC_GroupHalt.Done THEN
			_MC_GroupHalt.Execute := FALSE;
			SUPER^.M_StateComplete();
			_SeqHolding := 0;
		END_IF
		
END_CASE
END_METHOD

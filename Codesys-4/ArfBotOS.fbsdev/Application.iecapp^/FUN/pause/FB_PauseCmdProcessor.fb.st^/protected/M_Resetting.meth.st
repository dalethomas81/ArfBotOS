METHOD PROTECTED M_Resetting : BOOL

CASE SUPER^._SeqResetting OF
	0:
		_Timer_IN := FALSE;
		_SeqResetting := _SeqResetting + 1;
	1:
		IF NOT _Timer_IN THEN
			SUPER^.M_StateComplete();
			_SeqResetting := 0;
		END_IF
END_CASE
END_METHOD

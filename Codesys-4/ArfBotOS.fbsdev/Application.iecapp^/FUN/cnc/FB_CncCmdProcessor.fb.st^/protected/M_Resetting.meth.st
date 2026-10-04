METHOD PROTECTED M_Resetting : BOOL

CASE SUPER^._SeqResetting OF
	0:
		_PlanPath.M_Reset();
		_SeqResetting := _SeqResetting + 1;
	1:
		IF NOT _PlanPath.Error THEN
			SUPER^.M_StateComplete();
			_SeqResetting := 0;
		END_IF
END_CASE
END_METHOD

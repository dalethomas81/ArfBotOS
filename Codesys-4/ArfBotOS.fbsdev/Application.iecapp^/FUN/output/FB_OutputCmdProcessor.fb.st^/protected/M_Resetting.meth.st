METHOD PROTECTED M_Resetting : BOOL

CASE SUPER^._SeqResetting OF
	0:
		_SeqResetting := _SeqResetting + 1;
	1:
		SUPER^.M_StateComplete();
		_SeqResetting := 0;
END_CASE
END_METHOD

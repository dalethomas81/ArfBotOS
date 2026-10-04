METHOD PROTECTED M_Aborting : BOOL

CASE SUPER^._SeqAborting OF
	0:
		SUPER^._SeqAborting := SUPER^._SeqAborting + 1;
	1:
		SUPER^.M_StateComplete();
		SUPER^._SeqAborting := 0;
		
END_CASE
END_METHOD

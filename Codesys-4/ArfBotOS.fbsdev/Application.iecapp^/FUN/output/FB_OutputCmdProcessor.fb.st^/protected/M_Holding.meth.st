METHOD PROTECTED M_Holding : BOOL

CASE SUPER^._SeqHolding OF
	0:
		SUPER^._SeqHolding := SUPER^._SeqHolding + 1;
	1:
		SUPER^.M_StateComplete();
		SUPER^._SeqHolding := 0;
		
END_CASE
END_METHOD

METHOD PROTECTED M_Holding : BOOL

CASE _SeqHolding OF
	0:
		_SMC_Interpolator.bWaitAtNextStop := TRUE;
		SUPER^._SeqHolding := SUPER^._SeqHolding + 1;
	1:
		// TODO dale thomas this is not tested
		IF NOT _SMC_Interpolator.bWorking THEN
			SUPER^._SeqHolding := 0;
			SUPER^.M_StateComplete();
		END_IF
		
END_CASE
END_METHOD

METHOD PROTECTED M_Unholding : BOOL

CASE _SeqUnHolding OF
	0:
		_SMC_Interpolator.bWaitAtNextStop := FALSE;
		SUPER^._SeqUnHolding := SUPER^._SeqUnHolding + 1;
	1:
		// TODO dale thomas this is not tested
		IF _SMC_Interpolator.bWorking THEN
			SUPER^._SeqUnHolding := 0;
			SUPER^.M_StateComplete();
		END_IF
		
END_CASE
END_METHOD

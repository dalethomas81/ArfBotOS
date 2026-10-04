METHOD PROTECTED M_Aborting : BOOL

CASE _SeqAborting OF
	0:
		_Vision.M_Cancel();
		_SeqAborting := _SeqAborting + 1;
		
	1: 
		IF NOT _Vision.P_Busy THEN
			SUPER^.M_StateComplete();
			_SeqAborting := 0;
		END_IF
END_CASE
END_METHOD

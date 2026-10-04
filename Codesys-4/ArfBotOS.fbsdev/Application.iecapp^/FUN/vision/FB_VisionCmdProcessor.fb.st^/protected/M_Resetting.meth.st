METHOD PROTECTED M_Resetting : BOOL

CASE SUPER^._SeqResetting OF
	0:
		_Vision.M_Reset();
		_SeqResetting := _SeqResetting + 1;
	1:
		IF NOT _Vision.P_Error THEN
			//VisionStateText := 'RESET';
			SUPER^.M_StateComplete();
			_SeqResetting := 0;
		END_IF
END_CASE
		
END_METHOD

METHOD PROTECTED M_Resetting : BOOL

{warning disable C0371}
CASE SUPER^._SeqResetting OF
	0:
		GroupHandler.M_ResetGroup();
		_SeqResetting := _SeqResetting + 1;
	1:
		//IF NOT  THEN
			SUPER^.M_StateComplete();
			_SeqResetting := 0;
		//END_IF
END_CASE
{warning restore C0371}
END_METHOD

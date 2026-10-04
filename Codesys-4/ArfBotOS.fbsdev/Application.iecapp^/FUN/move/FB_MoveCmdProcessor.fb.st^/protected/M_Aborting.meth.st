METHOD PROTECTED M_Aborting : BOOL

{warning disable C0371}
CASE _SeqAborting OF
	0:
		GroupHandler.M_DisableGroup();
		_SeqAborting := _SeqAborting + 1;
	1:
		IF NOT GroupHandler.P_GroupEnabled THEN
			SUPER^.M_StateComplete();
			_SeqAborting := 0;
		END_IF
		
END_CASE
{warning restore C0371}
END_METHOD

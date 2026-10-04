METHOD PROTECTED M_Aborting : BOOL

CASE _SeqAborting OF
	0:
		_ProcessCommands.M_Abort();
		_SeqAborting := _SeqAborting + 1;
	1:
		IF _ProcessCommands.PropCurrentState = PACK_ML.State.Aborted THEN
			_SeqAborting := 0;
			SUPER^.M_StateComplete();
		END_IF
		
END_CASE

(*IF ItfActiveModule <> 0 THEN
	ItfActiveModule.M_Abort();
	IF ItfActiveModule.PropCurrentState = PACK_ML.State.Aborted THEN
		SUPER^.M_StateComplete();
	END_IF
ELSE
	SUPER^.M_StateComplete();
END_IF*)
END_METHOD

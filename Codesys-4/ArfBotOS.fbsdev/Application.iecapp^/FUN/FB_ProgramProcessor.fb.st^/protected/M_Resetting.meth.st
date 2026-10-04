METHOD PROTECTED M_Resetting : BOOL

CASE _SeqResetting OF
	0:
		_ProcessCommands.M_Home();
		_SeqResetting := _SeqResetting + 1;
	1:
		IF _ProcessCommands.PropCurrentState = PACK_ML.State.Idle THEN
			_SeqResetting := 0;
			SUPER^.M_StateComplete();
		END_IF
		
END_CASE

SUPER^.M_Reset();

(*IF ItfActiveModule <> 0 THEN
	CASE _Seq OF
		0:
			IF NOT ItfActiveModule.PropCurrentState = PACK_ML.State.Idle THEN
				ItfActiveModule.M_Home();
				_Seq := 1;
			ELSE
				SUPER^.M_StateComplete();
			END_IF
		1:
			IF ItfActiveModule.PropCurrentState = PACK_ML.State.Idle THEN
				_Seq := 0;
			END_IF
			
	END_CASE
	
ELSE
	SUPER^.M_StateComplete();
	_Seq := 0;
	
END_IF*)
END_METHOD

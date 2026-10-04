METHOD PROTECTED M_Resetting : BOOL

CASE _SeqResetting OF
	0:
		_ProcessMoveCommand.M_Home();
		_ProcessPauseCommand.M_Home();
		//_ProcessInputCommand.M_Home();
		_ProcessOutputCommand.M_Home();
		_ProcessVisionCommand.M_Home();
		_ProcessVarSetCommand.M_Home();
		_ProcessCncCommand.M_Home();
		_ProcessLogicCommand.M_Home();
        StatusText := 'Resetting';
		_SeqResetting := _SeqResetting + 1;
	1:
		IF _ProcessMoveCommand.PropCurrentState = PACK_ML.State.Idle
			AND _ProcessPauseCommand.PropCurrentState = PACK_ML.State.Idle 
			//AND _ProcessInputCommand.PropCurrentState = PACK_ML.State.Idle
			AND _ProcessOutputCommand.PropCurrentState = PACK_ML.State.Idle
			AND _ProcessVisionCommand.PropCurrentState = PACK_ML.State.Idle
			AND _ProcessVarSetCommand.PropCurrentState = PACK_ML.State.Idle
			AND _ProcessCncCommand.PropCurrentState = PACK_ML.State.Idle
			AND _ProcessLogicCommand.PropCurrentState = PACK_ML.State.Idle THEN
				_SeqResetting := 0;
                StatusText := 'Idle';
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

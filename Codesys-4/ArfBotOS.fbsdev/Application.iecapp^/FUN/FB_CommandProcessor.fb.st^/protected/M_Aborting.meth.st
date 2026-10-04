METHOD PROTECTED M_Aborting : BOOL

CASE _SeqAborting OF
	0:
		_ProcessMoveCommand.M_Abort();
		_ProcessPauseCommand.M_Abort();
		//_ProcessInputCommand.M_Abort();
		_ProcessOutputCommand.M_Abort();
		_ProcessVisionCommand.M_Abort();
		_ProcessVarSetCommand.M_Abort();
		_ProcessCncCommand.M_Abort();
		_ProcessLogicCommand.M_Abort();
		_SeqAborting := _SeqAborting + 1;
	1:
		IF _ProcessMoveCommand.PropCurrentState = PACK_ML.State.Aborted
			AND _ProcessPauseCommand.PropCurrentState = PACK_ML.State.Aborted 
			//AND _ProcessInputCommand.PropCurrentState = PACK_ML.State.Aborted
			AND _ProcessOutputCommand.PropCurrentState = PACK_ML.State.Aborted
			AND _ProcessVisionCommand.PropCurrentState = PACK_ML.State.Aborted
			AND _ProcessVarSetCommand.PropCurrentState = PACK_ML.State.Aborted
			AND _ProcessCncCommand.PropCurrentState = PACK_ML.State.Aborted
			AND _ProcessLogicCommand.PropCurrentState = PACK_ML.State.Aborted THEN
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

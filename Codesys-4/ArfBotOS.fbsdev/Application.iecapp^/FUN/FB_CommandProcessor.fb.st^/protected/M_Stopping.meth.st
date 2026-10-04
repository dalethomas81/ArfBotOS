METHOD PROTECTED M_Stopping : BOOL

IF ItfActiveModule <> 0 THEN
	ItfActiveModule.M_Stop();
	IF ItfActiveModule.PropCurrentState = PACK_ML.State.Idle 
		OR ItfActiveModule.PropCurrentState = PACK_ML.State.Stopped 
		OR ItfActiveModule.PropCurrentState = PACK_ML.State.Aborted 
		OR ItfActiveModule.PropCurrentState = PACK_ML.State.Complete THEN
			SUPER^.M_StateComplete();
	END_IF
	
ELSE
	SUPER^.M_StateComplete();
	
END_IF
END_METHOD

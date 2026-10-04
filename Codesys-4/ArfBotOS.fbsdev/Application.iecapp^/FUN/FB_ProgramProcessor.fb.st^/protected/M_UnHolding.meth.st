METHOD PROTECTED M_Unholding : BOOL

//
IF ItfActiveModule <> 0 THEN
	ItfActiveModule.M_Start();
	IF ItfActiveModule.PropCurrentState = PACK_ML.State.Execute THEN
		SUPER^.M_StateComplete();
	END_IF
	
ELSE
	SUPER^.M_StateComplete();
	
END_IF

//
_SeqMain := _STEP_WAIT_IDLE;
END_METHOD

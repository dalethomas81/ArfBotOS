METHOD PROTECTED M_Suspending : BOOL

IF ItfActiveModule <> 0 THEN
	IF ItfActiveModule.PropCurrentState = PACK_ML.State.Execute THEN
		ItfActiveModule.M_Hold();
		IF ItfActiveModule.PropCurrentState = PACK_ML.State.Held THEN
			SUPER^.M_StateComplete();
		END_IF
	ELSE
		SUPER^.M_StateComplete();
	END_IF
	
ELSE
	SUPER^.M_StateComplete();
	
END_IF
END_METHOD

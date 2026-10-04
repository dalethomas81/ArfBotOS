METHOD PROTECTED M_SetPackML : BOOL
VAR_INPUT
END_VAR

_UnitMode(sName := 'Normal Operation', 
			dwSupportedStates := PACK_ML.State.All,
			dwAllowsLeavingFromStates := PACK_ML.State.Stopped 
											OR PACK_ML.State.Aborted 
											OR PACK_ML.State.Idle, 
			dwAllowsEnteringIntoStates := PACK_ML.State.Stopped
											OR PACK_ML.State.Aborted 
											OR PACK_ML.State.Idle, 
			eInitialState := PACK_ML.State.Idle,
			xActive := TRUE);
END_METHOD

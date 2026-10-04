METHOD PROTECTED M_SetPackML : BOOL

// SUPER already does this but the default is eInitialState:=PACK_ML.State.Idle 
// and we want PACK_ML.State.Aborted for the Orchestrator
SUPER^._UnitMode(sName := 'Program Processor',
				eInitialState := PACK_ML.State.Aborted,
				xActive := TRUE);
END_METHOD

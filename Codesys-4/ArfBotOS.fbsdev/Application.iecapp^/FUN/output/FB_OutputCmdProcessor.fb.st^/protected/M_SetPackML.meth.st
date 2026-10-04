METHOD PROTECTED M_SetPackML : BOOL

SUPER^._UnitMode(	sName := 'Output Processor',
				eInitialState := PACK_ML.State.Idle,
				xActive := TRUE);
END_METHOD

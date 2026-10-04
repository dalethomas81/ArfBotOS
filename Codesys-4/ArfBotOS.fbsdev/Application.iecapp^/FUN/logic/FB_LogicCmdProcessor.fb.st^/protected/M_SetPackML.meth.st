METHOD PROTECTED M_SetPackML : BOOL

SUPER^._UnitMode(	sName := 'Logic Processor',
				eInitialState := PACK_ML.State.Idle,
				xActive := TRUE);
END_METHOD

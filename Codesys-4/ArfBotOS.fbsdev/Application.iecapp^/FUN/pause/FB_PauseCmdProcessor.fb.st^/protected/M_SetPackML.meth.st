METHOD PROTECTED M_SetPackML : BOOL

SUPER^._UnitMode(	sName := 'Pause Processor',
				eInitialState := PACK_ML.State.Idle,
				xActive := TRUE);
END_METHOD

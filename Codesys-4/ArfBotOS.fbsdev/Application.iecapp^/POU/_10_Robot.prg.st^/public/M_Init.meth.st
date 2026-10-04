METHOD PUBLIC M_Init : BOOL
VAR_INPUT
	DefaultParameters: BOOL;
END_VAR

IF DefaultParameters THEN
	PersistentVars.Flags[GVL_App.INIT_DRV_IDX].Value := FALSE;
END_IF
InitializeDrives := TRUE;
END_METHOD

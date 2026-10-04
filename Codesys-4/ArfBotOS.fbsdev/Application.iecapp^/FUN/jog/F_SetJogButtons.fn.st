FUNCTION F_SetJogButtons : INT
VAR_INPUT
	CoordinateSystem: SMC_COORD_SYSTEM;
	ABC_as_ACS: BOOL;
END_VAR
VAR
END_VAR

CASE CoordinateSystem OF
	SMC_COORD_SYSTEM.ACS:
		F_SetJogButtons := 0;
	SMC_COORD_SYSTEM.MCS:
		IF ABC_as_ACS THEN
			F_SetJogButtons := 2;
		ELSE
			F_SetJogButtons := 1;
		END_IF
	SMC_COORD_SYSTEM.PCS_1:
		IF ABC_as_ACS THEN
			F_SetJogButtons:= 2;
		ELSE
			F_SetJogButtons := 1;
		END_IF
	SMC_COORD_SYSTEM.PCS_2:
		IF ABC_as_ACS THEN
			F_SetJogButtons := 2;
		ELSE
			F_SetJogButtons := 1;
		END_IF
	SMC_COORD_SYSTEM.TCS:
		IF ABC_as_ACS THEN
			F_SetJogButtons := 2;
		ELSE
			F_SetJogButtons := 1;
		END_IF
	SMC_COORD_SYSTEM.WCS:
		IF ABC_as_ACS THEN
			F_SetJogButtons := 2;
		ELSE
			F_SetJogButtons := 1;
		END_IF
	ELSE
		F_SetJogButtons := 0;
		
END_CASE
END_FUNCTION

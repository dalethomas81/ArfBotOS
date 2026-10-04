FUNCTION F_DpadState : ST_Dpad
VAR_INPUT
	State: USINT;
END_VAR
VAR
END_VAR

CASE State OF
	0:
		F_DpadState.Up := TRUE;
		F_DpadState.Down := FALSE;
		F_DpadState.Left := FALSE;
		F_DpadState.Right := FALSE;
	1:
		F_DpadState.Up := TRUE;
		F_DpadState.Down := FALSE;
		F_DpadState.Left := FALSE;
		F_DpadState.Right := TRUE;
	2:
		F_DpadState.Up := FALSE;
		F_DpadState.Down := FALSE;
		F_DpadState.Left := FALSE;
		F_DpadState.Right := TRUE;
	3:
		F_DpadState.Up := FALSE;
		F_DpadState.Down := TRUE;
		F_DpadState.Left := FALSE;
		F_DpadState.Right := TRUE;
	4:
		F_DpadState.Up := FALSE;
		F_DpadState.Down := TRUE;
		F_DpadState.Left := FALSE;
		F_DpadState.Right := FALSE;
	5:
		F_DpadState.Up := FALSE;
		F_DpadState.Down := TRUE;
		F_DpadState.Left := TRUE;
		F_DpadState.Right := FALSE;
	6:
		F_DpadState.Up := FALSE;
		F_DpadState.Down := FALSE;
		F_DpadState.Left := TRUE;
		F_DpadState.Right := FALSE;
	7:
		F_DpadState.Up := TRUE;
		F_DpadState.Down := FALSE;
		F_DpadState.Left := TRUE;
		F_DpadState.Right := FALSE;
		
ELSE
	F_DpadState.Up := FALSE;
	F_DpadState.Down := FALSE;
	F_DpadState.Left := FALSE;
	F_DpadState.Right := FALSE;
		
END_CASE
END_FUNCTION

METHOD PUBLIC M_SetColor : BOOL
VAR_INPUT
	Red: INT:=0;
	Green: INT:=255;
	Blue: INT:=0;
END_VAR

IF Red>=0 AND_THEN Red<=255
	AND_THEN Green>=0 AND_THEN Green<=255
	AND_THEN Blue>=0 AND_THEN Blue<=255THEN
		M_SetColor := TRUE;
		_SetColor:=TRUE;
		_Red:=Red;
		_Green:=Green;
		_Blue:=Blue;
ELSE
	M_SetColor := FALSE;
END_IF
END_METHOD

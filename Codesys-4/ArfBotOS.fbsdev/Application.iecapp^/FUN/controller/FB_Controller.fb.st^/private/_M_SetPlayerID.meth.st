METHOD PRIVATE _M_SetPlayerID : BOOL
VAR_INPUT
END_VAR
VAR
	_Command: STRING(255);
END_VAR

IF NOT SUPER^._Busy AND _SetPlayerID THEN
	_SetPlayerID := FALSE;
	
	_Command := CONCAT('cmd: setPlayerID:', INT_TO_STRING(_PlayerID));
	
	SUPER^._M_Execute(Command:=_Command, Timeout:=5);
	SUPER^(); // call super here so other methods see it busy.
END_IF
END_METHOD

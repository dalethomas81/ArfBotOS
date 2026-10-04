METHOD PRIVATE _M_SetColor : BOOL
VAR_INPUT
END_VAR
VAR
	_Command: STRING(255);
END_VAR

IF NOT SUPER^._Busy AND_THEN _SetColor THEN
	_SetColor := FALSE;
	
	_Command := CONCAT('cmd: setColorI:', INT_TO_STRING(_Red));
	_Command := CONCAT(_Command, ',');
	_Command := CONCAT(_Command, INT_TO_STRING(_Green));
	_Command := CONCAT(_Command, ',');
	_Command := CONCAT(_Command, INT_TO_STRING(_Blue));
	
	SUPER^._M_Execute(Command:=_Command, Timeout:=5);
	SUPER^(); // call super here so other methods see it busy.
END_IF
END_METHOD

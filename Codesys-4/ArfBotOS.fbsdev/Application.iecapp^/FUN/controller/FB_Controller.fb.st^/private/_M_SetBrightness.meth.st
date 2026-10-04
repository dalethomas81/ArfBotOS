METHOD PRIVATE _M_SetBrightness : BOOL
VAR_INPUT
END_VAR

IF NOT SUPER^._Busy AND _SetBrightness THEN
	_SetBrightness := FALSE;
	
	_Command := CONCAT('cmd: setBrightness:', INT_TO_STRING(_Brightness));
	
	SUPER^._M_Execute(Command:=_Command, Timeout:=5);
	SUPER^(); // call super here so other methods see it busy.
END_IF
END_METHOD

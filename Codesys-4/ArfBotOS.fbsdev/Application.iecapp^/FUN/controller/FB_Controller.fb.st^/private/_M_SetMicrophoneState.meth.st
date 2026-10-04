METHOD PRIVATE _M_SetMicrophoneState : BOOL
VAR_INPUT
END_VAR
VAR
	_Command: STRING(255);
END_VAR

IF NOT SUPER^._Busy AND _SetMicrophoneState THEN
	_SetMicrophoneState := FALSE;
	
	_Command := CONCAT('cmd: setMicrophoneState:', BOOL_TO_STRING(_MicrophoneState));
	
	SUPER^._M_Execute(Command:=_Command, Timeout:=5);
	SUPER^(); // call super here so other methods see it busy.
END_IF;
END_METHOD

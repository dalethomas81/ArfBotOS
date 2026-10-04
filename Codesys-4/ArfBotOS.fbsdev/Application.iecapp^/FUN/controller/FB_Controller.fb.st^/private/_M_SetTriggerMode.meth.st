METHOD PRIVATE _M_SetTriggerMode : BOOL
VAR_INPUT
END_VAR
VAR
	_Command: STRING(255);
END_VAR

IF NOT SUPER^._Busy AND _SetTriggerMode THEN
	_SetTriggerMode := FALSE;
	
	IF _TriggerModeTrigger = E_ControllerTrigger.Left THEN
		_Command := 'cmd: setLeftTriggerMode:';
	ELSE
		_Command := 'cmd: setRightTriggerMode:';
	END_IF
	_Command := CONCAT(_Command, INT_TO_STRING(_TriggerModeMode));
	
	SUPER^._M_Execute(Command:=_Command, Timeout:=5);
	SUPER^(); // call super here so other methods see it busy.
END_IF
END_METHOD

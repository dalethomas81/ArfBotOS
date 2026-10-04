METHOD PRIVATE _M_SetTriggerForce : BOOL
VAR_INPUT
END_VAR
VAR
	_Command: STRING(255);
END_VAR

IF NOT SUPER^._Busy AND _SetTriggerForce THEN
	_SetTriggerForce := FALSE;
	
	IF _TriggerForceTrigger = E_ControllerTrigger.Left THEN
		_Command := 'cmd: setLeftTriggerForce:';
	ELSE
		_Command := 'cmd: setRightTriggerForce:';
	END_IF
	_Command := CONCAT(_Command, INT_TO_STRING(_TriggerForceParameter));
	_Command := CONCAT(_Command, ',');
	_Command := CONCAT(_Command, USINT_TO_STRING(_TriggerForceIntensity));
	
	SUPER^._M_Execute(Command:=_Command, Timeout:=5);
	SUPER^(); // call super here so other methods see it busy.
END_IF
END_METHOD

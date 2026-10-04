METHOD PRIVATE _M_SetMotorIntensity : BOOL
VAR_INPUT
END_VAR
VAR
	_Command: STRING(255);
END_VAR

IF NOT SUPER^._Busy AND _SetMotorIntensity THEN
	_SetMotorIntensity := FALSE;
	
	IF _Motor = E_ControllerMotor.Left THEN
		_Command := 'cmd: setLeftMotor:';
	ELSE
		_Command := 'cmd: setRightMotor:';
	END_IF
	_Command := CONCAT(_Command, USINT_TO_STRING(_MotorIntensity));
	_Command := CONCAT(_Command, ',');
	_Command := CONCAT(_Command, REAL_TO_STRING(_MotorIntensityDuration));
	
	SUPER^._M_Execute(Command:=_Command, Timeout:=5);
	SUPER^(); // call super here so other methods see it busy.
END_IF
END_METHOD

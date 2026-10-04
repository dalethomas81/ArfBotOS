METHOD PUBLIC M_SetMotorIntensity : BOOL
VAR_INPUT
	Motor: E_ControllerMotor;
	Intensity: USINT; // 0-255
	Duration: REAL; // s
END_VAR

IF Intensity >=0 AND_THEN Intensity <= 255 AND_THEN Duration <= 5000 THEN
	M_SetMotorIntensity := TRUE;
	_SetMotorIntensity := TRUE;
	_Motor := Motor;
	_MotorIntensity := Intensity;
	_MotorIntensityDuration := Duration;
ELSE
	M_SetMotorIntensity := FALSE;
END_IF
END_METHOD

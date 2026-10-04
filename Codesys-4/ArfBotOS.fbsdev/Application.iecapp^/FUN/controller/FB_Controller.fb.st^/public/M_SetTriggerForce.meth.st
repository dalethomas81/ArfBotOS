METHOD PUBLIC M_SetTriggerForce : BOOL
VAR_INPUT
	Trigger: E_ControllerTrigger;
	Parameter: E_ControllerTriggerForceParameter;
	Intensity: USINT; // 0-255
END_VAR

IF Intensity >= 0 AND Intensity <= 255 THEN
	M_SetTriggerForce := TRUE;
	_SetTriggerForce:=TRUE;
	_TriggerForceTrigger := Trigger;
	_TriggerForceParameter := Parameter;
	_TriggerForceIntensity := Intensity;
ELSE
	M_SetTriggerForce := FALSE;
END_IF
END_METHOD

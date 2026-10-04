METHOD PUBLIC M_SetTriggerMode : BOOL
VAR_INPUT
	Trigger: E_ControllerTrigger;
	Mode: E_ControllerTriggerMode;
END_VAR

M_SetTriggerMode := TRUE;
_SetTriggerMode := TRUE;
_TriggerModeTrigger := Trigger;
_TriggerModeMode := Mode;
END_METHOD

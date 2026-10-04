METHOD PROTECTED M_Error : BOOL
VAR_INPUT
	Description: STRING;
END_VAR

// suspend state machine
_UnitMode.Suspend();

//
Error := TRUE;

//
IF ErrorDescription = '' THEN
	ErrorDescription := Description;
ELSE // add a space and tack on the additional error description
	ErrorDescription := CONCAT(ErrorDescription, ' ');
	ErrorDescription := CONCAT(ErrorDescription, Description);
END_IF

GVL_App.sLastFault := ErrorDescription;
AlarmManager.AlarmGlobals.g_AlarmHandler.RaiseEvent(
    Alm_AlarmConfiguration_Alarmgroup_IDs.ID_Processors,
    Alm_Processors_Alarm_IDs.ID_ProcessorFault);
END_METHOD

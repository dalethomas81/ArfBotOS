METHOD EventCallback : SysTypes.RTS_IEC_RESULT
VAR_INPUT
	pEventParam : POINTER TO CmpEventMgr.EventParam;
END_VAR
VAR
	pLic : POINTER TO CmpMgr.EVTPARAM_CmpMgr_LicenseState;
END_VAR

EventCallback := 0;
IF pEventParam = 0 THEN
	RETURN;
END_IF
IF pEventParam^.EventId <> CmpMgr.EventIDs.EVT_CmpMgr_LicenseState THEN
	RETURN;
END_IF
IF pEventParam^.pParameter = 0 THEN
	RETURN;
END_IF

pLic := pEventParam^.pParameter;
eState := pLic^.state;
_xGotEvent := TRUE;
_tEvent := TIME();
IF pLic^.secondsToExpiration > 0 THEN
	_udiSecondsAtEvent := pLic^.secondsToExpiration;
ELSIF eState = CmpMgr.RUNTIME_LICENSE_STATE.DEMOMODE THEN
	_udiSecondsAtEvent := 7200;
ELSE
	_udiSecondsAtEvent := 0;
END_IF
udiSecondsLeft := _udiSecondsAtEvent;
END_METHOD

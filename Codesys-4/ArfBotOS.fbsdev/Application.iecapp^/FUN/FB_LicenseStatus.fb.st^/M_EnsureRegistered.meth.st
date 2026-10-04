METHOD PRIVATE M_EnsureRegistered
VAR
	tNow : TIME;
END_VAR

IF _xRegistered THEN
	RETURN;
END_IF

tNow := TIME();
IF _xTried AND ((tNow - _tRetry) < T#2S) THEN
	RETURN;
END_IF
_xTried := TRUE;
_tRetry := tNow;

IF _hEvent = SysTypes.RTS_INVALID_HANDLE THEN
	_hEvent := CmpEventMgr.EventOpen(
		EventId := CmpMgr.EventIDs.EVT_CmpMgr_LicenseState,
		CmpIdProvider := CmpMgr.ComponentID.CMPID_CmpMgr,
		Result := _result);
END_IF

IF _hEvent = SysTypes.RTS_INVALID_HANDLE THEN
	RETURN;
END_IF

_hCallback := CmpEventMgr.EventRegisterCallback2(
	hEvent := _hEvent,
	pICallback := THIS^,
	pUserParameter := 0,
	Result := _result);

IF _hCallback <> SysTypes.RTS_INVALID_HANDLE THEN
	_xRegistered := TRUE;
END_IF
END_METHOD

METHOD PRIVATE M_Unregister

IF _hCallback <> SysTypes.RTS_INVALID_HANDLE THEN
	_result := CmpEventMgr.EventUnregisterCallback(
		hEvent := _hEvent,
		hInterface := _hCallback);
END_IF
IF _hEvent <> SysTypes.RTS_INVALID_HANDLE THEN
	_result := CmpEventMgr.EventClose2(hEvent := _hEvent);
END_IF
_hEvent := SysTypes.RTS_INVALID_HANDLE;
_hCallback := SysTypes.RTS_INVALID_HANDLE;
_xRegistered := FALSE;
END_METHOD

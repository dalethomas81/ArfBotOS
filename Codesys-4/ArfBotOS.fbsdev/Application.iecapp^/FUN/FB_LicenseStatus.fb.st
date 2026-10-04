FUNCTION_BLOCK FB_LicenseStatus IMPLEMENTS CmpEventMgr.ICmpEventCallback
VAR_OUTPUT
	eState : CmpMgr.RUNTIME_LICENSE_STATE;
	udiSecondsLeft : UDINT;
	udiMinutesLeft : UDINT;
	tRemaining : TIME;
	sState : STRING(32);
	sBanner : STRING(80);
	xDemo : BOOL;
	xLicensed : BOOL;
	xShowBanner : BOOL;
END_VAR
VAR
	_hEvent : SysTypes.RTS_IEC_HANDLE := SysTypes.RTS_INVALID_HANDLE;
	_hCallback : SysTypes.RTS_IEC_HANDLE := SysTypes.RTS_INVALID_HANDLE;
	_result : SysTypes.RTS_IEC_RESULT;
	_xRegistered : BOOL;
	_xTried : BOOL;
	_xGotEvent : BOOL;
	_udiSecondsAtEvent : UDINT;
	_tEvent : TIME;
	_tRetry : TIME;
	_udiElapsed : UDINT;
	_sMin : STRING(11);
	_sSec : STRING(11);
	_udiSec : UDINT;
END_VAR

M_EnsureRegistered();
M_UpdateRemaining();
M_BuildBanner();
END_FUNCTION_BLOCK

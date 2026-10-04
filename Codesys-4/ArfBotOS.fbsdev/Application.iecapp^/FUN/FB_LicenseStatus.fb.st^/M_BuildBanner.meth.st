METHOD PRIVATE M_BuildBanner

CASE eState OF
	CmpMgr.RUNTIME_LICENSE_STATE.DEMOMODE:
		sState := 'DEMO';
	CmpMgr.RUNTIME_LICENSE_STATE.LICENSED:
		sState := 'LICENSED';
	CmpMgr.RUNTIME_LICENSE_STATE.LICENSE_LOST:
		sState := 'LICENSE LOST';
	CmpMgr.RUNTIME_LICENSE_STATE.TRIGGER_SHUTDOWN:
		sState := 'SHUTTING DOWN';
	CmpMgr.RUNTIME_LICENSE_STATE.SAFEMODE:
		sState := 'SAFE MODE';
ELSE
	sState := '';
END_CASE

IF NOT xShowBanner THEN
	sBanner := '';
	RETURN;
END_IF

IF eState = CmpMgr.RUNTIME_LICENSE_STATE.DEMOMODE THEN
	_udiSec := udiSecondsLeft MOD 60;
	_sMin := UDINT_TO_STRING(udiMinutesLeft);
	_sSec := UDINT_TO_STRING(_udiSec);
	sBanner := CONCAT('CODESYS RUNTIME DEMO LICENSE  ', _sMin);
	sBanner := CONCAT(sBanner, 'm ');
	sBanner := CONCAT(sBanner, _sSec);
	sBanner := CONCAT(sBanner, 's remaining');
ELSIF eState = CmpMgr.RUNTIME_LICENSE_STATE.TRIGGER_SHUTDOWN THEN
	sBanner := CONCAT('LICENSE SHUTDOWN  ', UDINT_TO_STRING(udiSecondsLeft));
	sBanner := CONCAT(sBanner, 's');
ELSE
	sBanner := sState;
END_IF
END_METHOD

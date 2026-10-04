METHOD PRIVATE M_UpdateRemaining

xDemo := (eState = CmpMgr.RUNTIME_LICENSE_STATE.DEMOMODE);
xLicensed := (eState = CmpMgr.RUNTIME_LICENSE_STATE.LICENSED);
xShowBanner := xDemo
	OR (eState = CmpMgr.RUNTIME_LICENSE_STATE.LICENSE_LOST)
	OR (eState = CmpMgr.RUNTIME_LICENSE_STATE.TRIGGER_SHUTDOWN)
	OR (eState = CmpMgr.RUNTIME_LICENSE_STATE.SAFEMODE);

IF xLicensed THEN
	udiSecondsLeft := 0;
	udiMinutesLeft := 0;
	tRemaining := T#0S;
	RETURN;
END_IF

IF _xGotEvent AND (xDemo OR (eState = CmpMgr.RUNTIME_LICENSE_STATE.TRIGGER_SHUTDOWN)) THEN
	_udiElapsed := TIME_TO_UDINT(TIME() - _tEvent) / 1000;
	IF _udiElapsed < _udiSecondsAtEvent THEN
		udiSecondsLeft := _udiSecondsAtEvent - _udiElapsed;
	ELSE
		udiSecondsLeft := 0;
	END_IF
END_IF

udiMinutesLeft := udiSecondsLeft / 60;
tRemaining := UDINT_TO_TIME(udiSecondsLeft * 1000);
END_METHOD

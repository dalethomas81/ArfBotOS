FUNCTION F_AxisErrorDetail : STRING(255)
VAR_IN_OUT
	Axis: AXIS_REF_SM3;
END_VAR
VAR_INPUT
	AxisName: STRING(8);
END_VAR
VAR
	_i: INT;
	_id: SMC_ERROR;
	_s: STRING(255);
	_root: STRING(180);
	_stop: STRING(180);
	_inst: STRING(80);
END_VAR

(* Decode the SoftMotion FB that actually put this axis in errorstop.
   AXIS_REF.fbeFBError[].wID is the real SMC_ERROR (following error, SW limit, ...).
   Code 183 (SMC_MS_AXIS_IN_ERRORSTOP) is only "Stop cannot be executed" after
   the axis is already faulted -- skip it when a root cause is present. *)
F_AxisErrorDetail := '';
IF (NOT Axis.bError) AND (Axis.nAxisState <> SMC_AXIS_STATE.errorstop) THEN
	RETURN;
END_IF

_s := AxisName;
_root := '';
_stop := '';

FOR _i := 0 TO g_SMC_NUMBER_FB_ERRORS DO
	_id := Axis.fbeFBError[_i].wID;
	IF _id <> SMC_NO_ERROR THEN
		_inst := Axis.fbeFBError[_i].strErrorInstance;
		IF (_id = SMC_MS_AXIS_IN_ERRORSTOP) OR (_id = SMC_H_AXIS_IN_ERRORSTOP) THEN
			IF _stop = '' THEN
				_stop := CONCAT(TO_STRING(_id), CONCAT(' ', SMC_ErrorString(_id, SMC_Language_Type.english)));
			END_IF
		ELSIF _root = '' THEN
			_root := CONCAT(TO_STRING(_id), CONCAT(' ', SMC_ErrorString(_id, SMC_Language_Type.english)));
			IF _inst <> '' THEN
				_root := CONCAT(_root, CONCAT(' [', CONCAT(_inst, ']')));
			END_IF
		END_IF
	END_IF
END_FOR

IF _root <> '' THEN
	_s := CONCAT(_s, CONCAT(' ', _root));
ELSIF _stop <> '' THEN
	_s := CONCAT(_s, CONCAT(' ', _stop));
ELSE
	_s := CONCAT(_s, ' errorstop');
END_IF

IF Axis.dwErrorID <> 0 THEN
	_s := CONCAT(_s, CONCAT(' drive ', DWORD_TO_STRING(Axis.dwErrorID)));
END_IF

F_AxisErrorDetail := _s;
END_FUNCTION

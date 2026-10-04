METHOD PRIVATE _M_CheckOffsets : BOOL
VAR_INPUT
END_VAR
VAR
	_i, _j, _k: INT;
END_VAR

IF _SMC_GroupJog2.Enable THEN
	FOR _j:=0 TO GVL_App.MCS_POS_IDX-GVL_App.TCP_POS_IDX BY 1 DO
		_OffsetExist[_j] := FALSE;
	END_FOR
	FOR _i := GVL_App.TCP_POS_IDX TO GVL_App.MCS_POS_IDX BY 1 DO
		IF StoredPositions[_i].Position.X <> 0
			OR_ELSE StoredPositions[_i].Position.Y <> 0
			OR_ELSE StoredPositions[_i].Position.Z <> 0
			OR_ELSE StoredPositions[_i].Position.A <> 0
			OR_ELSE StoredPositions[_i].Position.B <> 0
			OR_ELSE StoredPositions[_i].Position.C <> 0 THEN
				_OffsetExist[_i - GVL_App.TCP_POS_IDX] := TRUE;
		END_IF
	END_FOR
END_IF
END_METHOD

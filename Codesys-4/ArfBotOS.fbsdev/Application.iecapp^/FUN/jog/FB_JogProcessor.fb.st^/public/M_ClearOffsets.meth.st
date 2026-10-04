METHOD PUBLIC M_ClearOffsets : BOOL
VAR_INPUT
END_VAR

FOR _j:=0 TO 3 BY 1 DO
	_OffsetExist[_j] := FALSE;
END_FOR
FOR _i := GVL_App.TCP_POS_IDX TO GVL_App.MCS_POS_IDX BY 1 DO
	StoredPositions[_i].Position.X := 0.0;
	StoredPositions[_i].Position.Y := 0.0;
	StoredPositions[_i].Position.Z := 0.0;
	StoredPositions[_i].Position.A := 0.0;
	StoredPositions[_i].Position.B := 0.0;
	StoredPositions[_i].Position.C := 0.0;
END_FOR
END_METHOD

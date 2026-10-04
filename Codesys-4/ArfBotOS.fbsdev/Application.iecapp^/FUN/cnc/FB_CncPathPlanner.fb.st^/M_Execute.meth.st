METHOD PUBLIC M_Execute : BOOL
VAR_INPUT
	Filename: STRING := 'CNC_FromFile.cnc';
END_VAR

_Filename := CONCAT(_BaseCncDirectory, Filename);
_Execute:=TRUE;
Done := FALSE;
END_METHOD

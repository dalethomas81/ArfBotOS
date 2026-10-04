METHOD PUBLIC M_Cancel : BOOL
VAR_INPUT
END_VAR

IF _WorkerUid <> '' THEN
	M_Cancel := TRUE;
	_Command := 'kill: ';
	_Command := CONCAT(_Command, _WorkerUid);
	_WorkerUid := ''; // lets make sure to forget this uid in case its used again for something else
	_Step := 0;
	_Execute := TRUE;
	
ELSE
	M_Cancel := FALSE;
	
END_IF
END_METHOD

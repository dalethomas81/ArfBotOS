METHOD INTERNAL _M_Write : BOOL
VAR_INPUT
	Data: STRING(255);
END_VAR

IF bConnected AND NOT bBusyWrite THEN
	bBusyWrite := TRUE; // do this as a workaround to show "busy" ASAP to let callers know to hold back
	_Write:=TRUE;
	_DataToWrite:=Data;
	_M_Write := TRUE;
	
ELSE
	_M_Write := FALSE;
	
END_IF
END_METHOD

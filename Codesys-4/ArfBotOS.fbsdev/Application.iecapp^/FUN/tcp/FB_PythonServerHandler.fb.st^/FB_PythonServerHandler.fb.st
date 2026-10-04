FUNCTION_BLOCK FB_PythonServerHandler
VAR_INPUT
	sIpAddress: STRING(19):='127.0.0.1';
	uPort: UINT:=50011;
END_VAR
VAR_OUTPUT
END_VAR
VAR
	_Done: BOOL;
	_Busy: BOOL;
	_Error: BOOL;
	_ErrorDescription: STRING;
	_Response: STRING(255);
	_WorkerUid: STRING;
	_Execute: BOOL;
	_Reset: BOOL:=FALSE;
	_ResetRtrig: R_TRIG;
	_Step: INT;
	_Command: STRING(255);
	_ReadRecordLast: DINT;
	_TcpClient: FB_TcpClient;
	_TimeoutTimer: TON:=(PT:=T#120S);
END_VAR

_TimeoutTimer(IN:=_Step > 0);
IF _TimeoutTimer.Q THEN
	_Step := 0;
	_Error := TRUE;
	_ErrorDescription := 'timed out waiting for response.';
END_IF

_ResetRtrig(CLK:=_Reset);
IF _ResetRtrig.Q THEN
	_Reset := FALSE;
	_Step := 0;
END_IF

_Done := _Step = 0;
_Busy := NOT _Done;

CASE _Step OF
	0: // IDLE
		IF _Execute THEN
			_Execute := FALSE;
			IF _TcpClient._M_Write(Data:=_Command) THEN
				_ReadRecordLast := _TcpClient.dRecordRead;
				_Step := _Step + 1;
			ELSE
				_Step := 0;
			END_IF
		END_IF
	
	1: // WAIT
		IF _TcpClient.dRecordRead <> _ReadRecordLast THEN
			_ReadRecordLast := _TcpClient.dRecordRead;
			IF find(_TcpClient.strRead, 'res: ') = 1 THEN
				MemUtils.MemSet(
					pbyBuffer:=ADR(_Response),
					byValue:=0,
					dwSize:=SIZEOF(_Response));
				_Response := _TcpClient.strRead;
				_Step := 0;
				
			ELSIF find(_TcpClient.strRead, 'worker uid: ') = 1 THEN
				MemUtils.MemSet(
					pbyBuffer:=ADR(_WorkerUid),
					byValue:=0,
					dwSize:=SIZEOF(_WorkerUid));
				_WorkerUid := DELETE(STR:=_TcpClient.strRead, LEN:=12, POS:=1);
				
			ELSIF find(_TcpClient.strRead, 'command uid: ') = 1 THEN
				MemUtils.MemSet(
					pbyBuffer:=ADR(_WorkerUid),
					byValue:=0,
					dwSize:=SIZEOF(_WorkerUid));
				_WorkerUid := DELETE(STR:=_TcpClient.strRead, LEN:=12, POS:=1);
				
			END_IF
		END_IF
		
END_CASE

_TcpClient(
	bEnable:= TRUE, 
	strIpAddress:= sIpAddress, 
	uPort:= uPort,
	bBusyWrite=> , 
	bDoneWrite=> , 
	bBusyRead=> , 
	dRecordRead=> , 
	strRead=> );
END_FUNCTION_BLOCK

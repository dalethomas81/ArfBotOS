FUNCTION_BLOCK FB_Controller EXTENDS FB_PythonServerHandler
VAR_INPUT
END_VAR
VAR_OUTPUT
END_VAR
VAR
	Controller: ST_Controller;
	ControllerStates: ARRAY [0..254] OF BYTE;
	ControllerStatesTemp: ARRAY [0..254] OF BYTE;
	ReadStringTemp: STRING(255);
	RecordLast: DINT;
	ReconnectTimer: TON:=(PT:=T#5S);
	
	_SetColor: BOOL;
	_Red: INT:=0;
	_Green: INT:=255;
	_Blue: INT:=0;
	
	_SetMicrophoneState: BOOL;
	_MicrophoneState: BOOL;
	
	_SetPlayerID: BOOL;
	_PlayerID: E_ControllerPlayerID;
	
	_SetBrightness: BOOL;
	_Brightness: E_ControllerBrightness;
	
	_SetMotorIntensity: BOOL;
	_Motor: E_ControllerMotor;
	_MotorIntensity: USINT; // 0-255
	_MotorIntensityDuration: REAL; // ms
	
	_SetTriggerMode: BOOL;
	_TriggerModeTrigger: E_ControllerTrigger;
	_TriggerModeMode: E_ControllerTriggerMode;
	
	_SetTriggerForce: BOOL;
	_TriggerForceTrigger: E_ControllerTrigger;
	_TriggerForceParameter: E_ControllerTriggerForceParameter;
	_TriggerForceIntensity: USINT;
	
	_i, _j, _k: INT;
	FirstScan: BOOL:=TRUE;
END_VAR
VAR CONSTANT
	States: STRING:='states: ';
END_VAR

SUPER^();

// copy over controller states when a new message is received
IF SUPER^.P_RecordNumber <> RecordLast THEN
	RecordLast := SUPER^.P_RecordNumber;
	IF FIND(SUPER^.P_RawReadString, States) > 0 THEN // TODO this is getting messy. need to overhaul how this works
		ControllerStatesTemp := SUPER^.P_RawReadByteArray;
		FOR _i:=0 TO UINT_TO_INT(SIZEOF(ControllerStates)) - LEN(States) BY 1 DO
			ControllerStates[_i] := ControllerStatesTemp[_i + LEN(States)];
		END_FOR
		
	END_IF
END_IF

//
_M_EvaluateControllerStates(States:=ControllerStates);
_M_SetColor();
_M_SetMicrophoneState();
_M_SetPlayerID();
_M_SetBrightness();
_M_SetMotorIntensity();
_M_SetTriggerMode();
_M_SetTriggerForce();

// try to reconnect to the server when data stops?
// leave this commented out until we have a better condition
// the edge case is when the controller is connected via bluetooth 
// but for some reason the server is not connected to it.
// maybe work this issue out on the server side.
(*ReconnectTimer(IN:= RecordLast=SUPER^.P_RecordNumber);
IF ReconnectTimer.Q THEN
	ReconnectTimer(IN:=FALSE);
	SUPER^.M_Reconnect();
END_IF*)

FirstScan:=FALSE;
END_FUNCTION_BLOCK

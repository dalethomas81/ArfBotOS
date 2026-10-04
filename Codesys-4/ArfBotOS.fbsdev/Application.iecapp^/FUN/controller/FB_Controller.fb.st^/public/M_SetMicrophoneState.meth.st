METHOD PUBLIC M_SetMicrophoneState : BOOL
VAR_INPUT
	State: BOOL;
END_VAR

M_SetMicrophoneState := TRUE;
_MicrophoneState := State;
_SetMicrophoneState := TRUE;
END_METHOD

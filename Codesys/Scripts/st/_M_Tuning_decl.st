METHOD PRIVATE _M_Tuning : BOOL
VAR_INPUT
END_VAR
VAR_STAT
	_MoveDelayTimer: ARRAY[0..5] OF TON;
	_TraceStartFtrig: ARRAY[0..5] OF F_TRIG;
	_MoveBusyFtrig: ARRAY[0..5] OF F_TRIG;
	_MoveBusyRtrig: ARRAY[0..5] OF R_TRIG;
	_MoveDelayTime: INT:=10; // ms
	_eDtState: ARRAY[0..5] OF INT; (* 0 idle, 1 wait cruise, 2 sampling, 3 done *)
	_bEstEnable: ARRAY[0..5] OF BOOL;
	_bDtFinish: ARRAY[0..5] OF BOOL;
	_aDtSample: ARRAY[0..5, 0..63] OF LREAL;
	_nSamples: ARRAY[0..5] OF INT;
	_fSetVel: LREAL;
	_fCruiseVel: LREAL;
	_fMedian: LREAL;
	_fSwap: LREAL;
	_fDtMin: LREAL := 0.1; (* SoftMotion rejects D = 0 *)
	_fDtMax: LREAL := 20.0; (* PosControl delay-buffer limit *)
	_fCruiseRatio: LREAL := 0.9;
	_nSampleMax: INT := 64;
	_i, _j, _k: INT;
END_VAR

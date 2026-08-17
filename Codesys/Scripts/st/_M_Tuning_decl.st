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
	_bKpMode: ARRAY[0..5] OF BOOL;
	_bKpActive: ARRAY[0..5] OF BOOL;
	_bKpFinish: ARRAY[0..5] OF BOOL;
	_bKpApproached: ARRAY[0..5] OF BOOL;
	_fKpDist: ARRAY[0..5] OF LREAL;
	_fKpTarget: ARRAY[0..5] OF LREAL;
	_fKpActStart: ARRAY[0..5] OF LREAL;
	_fKpActMin: ARRAY[0..5] OF LREAL;
	_fKpActMax: ARRAY[0..5] OF LREAL;
	_fKpActLast: ARRAY[0..5] OF LREAL;
	_fKpPrevTgtErr: ARRAY[0..5] OF LREAL;
	_fKpPrevActDelta: ARRAY[0..5] OF LREAL;
	_fKpCruiseSum: ARRAY[0..5] OF LREAL;
	_nKpCruiseN: ARRAY[0..5] OF INT;
	_nKpSettle: ARRAY[0..5] OF INT;
	_bKpSettleInit: ARRAY[0..5] OF BOOL;
	_fKpSettleLeg: ARRAY[0..5] OF LREAL;
	_fSetPos: LREAL;
	_fActPos: LREAL;
	_fLag: LREAL;
	_fKpNow: LREAL;
	_fKpMin: LREAL := 0.0001;
	_fKpMax: LREAL := 2.0;
	_fKpRaise: LREAL := 1.5;
	_fKpLower: LREAL := 0.9;
	_fKpLagNoise: LREAL := 0.02;
	_fKpDeltaMin: LREAL := 0.003;
	_fKpBounceMin: LREAL := 0.08;
	_fKpApproachFrac: LREAL := 0.3;
	_fKpReachBand: LREAL := 0.15;
	_nKpSettleCycles: INT := 200;
	_i, _j, _k: INT;
END_VAR

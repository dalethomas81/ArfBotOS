// MainTask interval is the SoftMotion bus cycle that D is counted in.
// TaskInfo.dwInterval is microseconds (simulation writes 1000).
IF GVL.dwMainInterval > 0 THEN
	fCycleTimeMs := DWORD_TO_LREAL(GVL.dwMainInterval) / 1000.0;
ELSE
	fCycleTimeMs := 1.0;
END_IF

IF FirstScan THEN
	FOR _i:=0 TO 5 BY 1 DO
		sDeadTimeHelp[_i] := 'SMC_EstimateDeadTime: START TUNE at Kp=0. Suggested D is the median of constant-velocity samples.';
	END_FOR
END_IF

// initial values
IF SetTuningMoveValues AND_THEN PersistentVars.Flags[GVL.INIT_DRV_IDX].Value THEN
	SetTuningMoveValues := FALSE;
	
	MoveRelative[0].Velocity := IoConfig_Globals.J1.fSWMaxVelocity;
	MoveRelative[0].Acceleration := IoConfig_Globals.J1.fSWMaxAcceleration;
	MoveRelative[0].Deceleration := IoConfig_Globals.J1.fSWMaxDeceleration;
	MoveRelative[0].Jerk := IoConfig_Globals.J1.fSWMaxJerk;
	MoveRelative[0].Distance := MoveDistance;
	
	MoveRelative[1].Velocity := IoConfig_Globals.J2.fSWMaxVelocity;
	MoveRelative[1].Acceleration := IoConfig_Globals.J2.fSWMaxAcceleration;
	MoveRelative[1].Deceleration := IoConfig_Globals.J2.fSWMaxDeceleration;
	MoveRelative[1].Jerk := IoConfig_Globals.J2.fSWMaxJerk;
	MoveRelative[1].Distance := MoveDistance;
	
	MoveRelative[2].Velocity := IoConfig_Globals.J3.fSWMaxVelocity;
	MoveRelative[2].Acceleration := IoConfig_Globals.J3.fSWMaxAcceleration;
	MoveRelative[2].Deceleration := IoConfig_Globals.J3.fSWMaxDeceleration;
	MoveRelative[2].Jerk := IoConfig_Globals.J3.fSWMaxJerk;
	MoveRelative[2].Distance := MoveDistance;
	
	MoveRelative[3].Velocity := IoConfig_Globals.J4.fSWMaxVelocity;
	MoveRelative[3].Acceleration := IoConfig_Globals.J4.fSWMaxAcceleration;
	MoveRelative[3].Deceleration := IoConfig_Globals.J4.fSWMaxDeceleration;
	MoveRelative[3].Jerk := IoConfig_Globals.J4.fSWMaxJerk;
	MoveRelative[3].Distance := MoveDistance;
	
	MoveRelative[4].Velocity := IoConfig_Globals.J5.fSWMaxVelocity;
	MoveRelative[4].Acceleration := IoConfig_Globals.J5.fSWMaxAcceleration;
	MoveRelative[4].Deceleration := IoConfig_Globals.J5.fSWMaxDeceleration;
	MoveRelative[4].Jerk := IoConfig_Globals.J5.fSWMaxJerk;
	MoveRelative[4].Distance := MoveDistance;
	
	MoveRelative[5].Velocity := IoConfig_Globals.J6.fSWMaxVelocity;
	MoveRelative[5].Acceleration := IoConfig_Globals.J6.fSWMaxAcceleration;
	MoveRelative[5].Deceleration := IoConfig_Globals.J6.fSWMaxDeceleration;
	MoveRelative[5].Jerk := IoConfig_Globals.J6.fSWMaxJerk;
	MoveRelative[5].Distance := MoveDistance;
END_IF

// trace activation
FOR _i:=0 TO 5 BY 1 DO
	
	IF TuneStart[_i] THEN
		TuneStart[_i] := FALSE;
		TraceStart[_i] := TRUE; // trace will handshake this bit low
		GroupHandler.M_EnableGroup();
		bDeadTimeSuggested[_i] := FALSE;
		fMeasuredLagMs[_i] := 0.0;
		fEstimatedDeadTimeRaw[_i] := 0.0;
		fSuggestedDeadTime[_i] := 0.0;
		nDeadTimeSamples[_i] := 0;
		_eDtState[_i] := 0;
		_bEstEnable[_i] := FALSE;
		_bDtFinish[_i] := FALSE;
		_nSamples[_i] := 0;
		sDeadTimeHelp[_i] := 'SMC_EstimateDeadTime: waiting for constant velocity. Kp=0 gives the cleanest estimate.';
	END_IF
	
	// copy a valid suggestion into the DeadTime spin; WRITE PARAMETERS still applies it
	IF UseSuggestedD[_i] THEN
		UseSuggestedD[_i] := FALSE;
		IF bDeadTimeSuggested[_i] THEN
			PersistentVars.PositionControllers[_i].fDeadTime := fSuggestedDeadTime[_i];
			sDeadTimeHelp[_i] := 'Copied suggested D into DeadTime. Press WRITE PARAMETERS to apply.';
		ELSE
			sDeadTimeHelp[_i] := 'No suggested D yet. Press START TUNE first.';
		END_IF
	END_IF
	
	// TraceStart will clear after handshake. this means the trace has started collecting samples
	// start the move delay timer to capture samples before the move
	_TraceStartFtrig[_i](CLK:=TraceStart[_i]);
	IF _TraceStartFtrig[_i].Q THEN
		_MoveDelayTimer[_i].IN:=TRUE;
	END_IF
	
	// once the move delay expires, start the move
	_MoveDelayTimer[_i](PT:=INT_TO_TIME(_MoveDelayTime));
	IF _MoveDelayTimer[_i].Q THEN
		_MoveDelayTimer[_i].IN:=FALSE;
		MoveRelative[_i].Execute := TRUE;
	END_IF
	
	_MoveBusyFtrig[_i](CLK:=MoveRelative[_i].Busy);
	IF _MoveBusyFtrig[_i].Q THEN
		MoveRelative[_i].Execute := FALSE;
		MoveRelative[_i].Distance := MoveRelative[_i].Distance * -1.0; // reverse direction
		_bEstEnable[_i] := FALSE;
		IF _eDtState[_i] = 2 THEN
			_bDtFinish[_i] := TRUE;
		ELSIF _eDtState[_i] = 1 THEN
			_eDtState[_i] := 0;
			sDeadTimeHelp[_i] := 'Move ended before constant velocity. Increase MoveDistance or reduce acceleration.';
		END_IF
	END_IF

	CASE _i OF
		0: _fSetVel := IoConfig_Globals.J1.fSetVelocity;
		1: _fSetVel := IoConfig_Globals.J2.fSetVelocity;
		2: _fSetVel := IoConfig_Globals.J3.fSetVelocity;
		3: _fSetVel := IoConfig_Globals.J4.fSetVelocity;
		4: _fSetVel := IoConfig_Globals.J5.fSetVelocity;
		5: _fSetVel := IoConfig_Globals.J6.fSetVelocity;
	END_CASE
	
	_MoveBusyRtrig[_i](CLK:=MoveRelative[_i].Busy);
	IF _MoveBusyRtrig[_i].Q THEN
		_nSamples[_i] := 0;
		nDeadTimeSamples[_i] := 0;
		_bDtFinish[_i] := FALSE;
		_eDtState[_i] := 1;
	END_IF
	
	_fCruiseVel := MoveRelative[_i].Velocity * _fCruiseRatio;
	IF _eDtState[_i] = 1 THEN
		IF ABS(_fSetVel) >= _fCruiseVel THEN
			_bEstEnable[_i] := TRUE;
			_eDtState[_i] := 2;
			sDeadTimeHelp[_i] := 'SMC_EstimateDeadTime: sampling at constant velocity...';
		END_IF
	END_IF
	
	IF _eDtState[_i] = 2 THEN
		IF _bEstEnable[_i] AND EstimateDeadTime[_i].Valid AND (_nSamples[_i] < _nSampleMax) THEN
			_aDtSample[_i, _nSamples[_i]] := ABS(EstimateDeadTime[_i].DeadTimeCycles);
			_nSamples[_i] := _nSamples[_i] + 1;
			nDeadTimeSamples[_i] := _nSamples[_i];
			IF _nSamples[_i] >= _nSampleMax THEN
				_bDtFinish[_i] := TRUE;
			END_IF
		END_IF
	END_IF
	
	IF _bDtFinish[_i] THEN
		_bDtFinish[_i] := FALSE;
		_bEstEnable[_i] := FALSE;
		_eDtState[_i] := 3;
		IF _nSamples[_i] < 1 THEN
			_eDtState[_i] := 0;
			sDeadTimeHelp[_i] := 'SMC_EstimateDeadTime never went Valid. Check enable and that the axis reached cruise.';
		ELSE
			// median of collected DeadTimeCycles
			FOR _j:=0 TO (_nSamples[_i] - 2) BY 1 DO
				FOR _k:=(_j + 1) TO (_nSamples[_i] - 1) BY 1 DO
					IF _aDtSample[_i, _k] < _aDtSample[_i, _j] THEN
						_fSwap := _aDtSample[_i, _j];
						_aDtSample[_i, _j] := _aDtSample[_i, _k];
						_aDtSample[_i, _k] := _fSwap;
					END_IF
				END_FOR
			END_FOR
			_fMedian := ABS(_aDtSample[_i, _nSamples[_i] / 2]);
			fEstimatedDeadTimeRaw[_i] := _fMedian;
			fMeasuredLagMs[_i] := _fMedian * fCycleTimeMs;
			IF _fMedian < _fDtMin THEN
				fSuggestedDeadTime[_i] := _fDtMin;
			ELSIF _fMedian > _fDtMax THEN
				fSuggestedDeadTime[_i] := _fDtMax;
			ELSE
				fSuggestedDeadTime[_i] := _fMedian;
			END_IF
			bDeadTimeSuggested[_i] := TRUE;
			sDeadTimeHelp[_i] := 'SMC_EstimateDeadTime median ';
			sDeadTimeHelp[_i] := CONCAT(sDeadTimeHelp[_i], TO_STRING(fEstimatedDeadTimeRaw[_i]));
			sDeadTimeHelp[_i] := CONCAT(sDeadTimeHelp[_i], ' cycles (');
			sDeadTimeHelp[_i] := CONCAT(sDeadTimeHelp[_i], TO_STRING(fMeasuredLagMs[_i]));
			sDeadTimeHelp[_i] := CONCAT(sDeadTimeHelp[_i], ' ms, n=');
			sDeadTimeHelp[_i] := CONCAT(sDeadTimeHelp[_i], INT_TO_STRING(_nSamples[_i]));
			IF fEstimatedDeadTimeRaw[_i] > _fDtMax THEN
				sDeadTimeHelp[_i] := CONCAT(sDeadTimeHelp[_i], '). Clamped to D 20 (PosControl max). WRITE PARAMETERS to apply.');
			ELSE
				sDeadTimeHelp[_i] := CONCAT(sDeadTimeHelp[_i], '). Suggested D ');
				sDeadTimeHelp[_i] := CONCAT(sDeadTimeHelp[_i], TO_STRING(fSuggestedDeadTime[_i]));
				sDeadTimeHelp[_i] := CONCAT(sDeadTimeHelp[_i], '. WRITE PARAMETERS to apply.');
			END_IF
		END_IF
	END_IF

END_FOR


// official SoftMotion estimator: enable only while at constant velocity
EstimateDeadTime[0](Axis:= IoConfig_Globals.J1, Enable:= _bEstEnable[0]);
EstimateDeadTime[1](Axis:= IoConfig_Globals.J2, Enable:= _bEstEnable[1]);
EstimateDeadTime[2](Axis:= IoConfig_Globals.J3, Enable:= _bEstEnable[2]);
EstimateDeadTime[3](Axis:= IoConfig_Globals.J4, Enable:= _bEstEnable[3]);
EstimateDeadTime[4](Axis:= IoConfig_Globals.J5, Enable:= _bEstEnable[4]);
EstimateDeadTime[5](Axis:= IoConfig_Globals.J6, Enable:= _bEstEnable[5]);

// move relative
MoveRelative[0](
	Axis:= IoConfig_Globals.J1, 
	Execute:= , 
	Distance:= , 
	Velocity:= , 
	Acceleration:= , 
	Deceleration:= , 
	Jerk:= , 
	BufferMode:= , 
	Done=> , 
	Busy=> , 
	Active=> , 
	CommandAborted=> , 
	Error=> , 
	ErrorID=> );
MoveRelative[1](
	Axis:= IoConfig_Globals.J2, 
	Execute:= , 
	Distance:= , 
	Velocity:= , 
	Acceleration:= , 
	Deceleration:= , 
	Jerk:= , 
	BufferMode:= , 
	Done=> , 
	Busy=> , 
	Active=> , 
	CommandAborted=> , 
	Error=> , 
	ErrorID=> );
MoveRelative[2](
	Axis:= IoConfig_Globals.J3, 
	Execute:= , 
	Distance:= , 
	Velocity:= , 
	Acceleration:= , 
	Deceleration:= , 
	Jerk:= , 
	BufferMode:= , 
	Done=> , 
	Busy=> , 
	Active=> , 
	CommandAborted=> , 
	Error=> , 
	ErrorID=> );
MoveRelative[3](
	Axis:= IoConfig_Globals.J4, 
	Execute:= , 
	Distance:= , 
	Velocity:= , 
	Acceleration:= , 
	Deceleration:= , 
	Jerk:= , 
	BufferMode:= , 
	Done=> , 
	Busy=> , 
	Active=> , 
	CommandAborted=> , 
	Error=> , 
	ErrorID=> );
MoveRelative[4](
	Axis:= IoConfig_Globals.J5, 
	Execute:= , 
	Distance:= , 
	Velocity:= , 
	Acceleration:= , 
	Deceleration:= , 
	Jerk:= , 
	BufferMode:= , 
	Done=> , 
	Busy=> , 
	Active=> , 
	CommandAborted=> , 
	Error=> , 
	ErrorID=> );
MoveRelative[5](
	Axis:= IoConfig_Globals.J6, 
	Execute:= , 
	Distance:= , 
	Velocity:= , 
	Acceleration:= , 
	Deceleration:= , 
	Jerk:= , 
	BufferMode:= , 
	Done=> , 
	Busy=> , 
	Active=> , 
	CommandAborted=> , 
	Error=> , 
	ErrorID=> );

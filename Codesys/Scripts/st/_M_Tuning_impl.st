// MainTask interval is the SoftMotion bus cycle that D is counted in.
// TaskInfo.dwInterval is microseconds (simulation writes 1000).
IF GVL.dwMainInterval > 0 THEN
	fCycleTimeMs := DWORD_TO_LREAL(GVL.dwMainInterval) / 1000.0;
ELSE
	fCycleTimeMs := 1.0;
END_IF

IF FirstScan THEN
	FOR _i:=0 TO 5 BY 1 DO
		sDeadTimeHelp[_i] := 'Kp=0 then START TUNE to estimate D. Kp>0 then START TUNE to score Kp.';
		sKpHelp[_i] := 'Set Kp above 0, WRITE PARAMETERS, then START TUNE to score overshoot and lag.';
		sKpVerdict[_i] := '';
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
		_bKpMode[_i] := PersistentVars.PositionControllers[_i].fKp > 0.0;
		bDeadTimeSuggested[_i] := FALSE;
		fMeasuredLagMs[_i] := 0.0;
		fEstimatedDeadTimeRaw[_i] := 0.0;
		fSuggestedDeadTime[_i] := 0.0;
		nDeadTimeSamples[_i] := 0;
		_eDtState[_i] := 0;
		_bEstEnable[_i] := FALSE;
		_bDtFinish[_i] := FALSE;
		_nSamples[_i] := 0;
		bKpSuggested[_i] := FALSE;
		fSuggestedKp[_i] := 0.0;
		fKpPeakLag[_i] := 0.0;
		fKpCruiseLag[_i] := 0.0;
		fKpFinalLag[_i] := 0.0;
		fKpOvershoot[_i] := 0.0;
		fKpUndershoot[_i] := 0.0;
		nKpReversals[_i] := 0;
		nKpVelReversals[_i] := 0;
		sKpVerdict[_i] := '';
		_bKpActive[_i] := FALSE;
		_bKpFinish[_i] := FALSE;
		_bKpApproached[_i] := FALSE;
		_nKpSettle[_i] := 0;
		_bKpSettleInit[_i] := FALSE;
		_fKpSettleLeg[_i] := 0.0;
		IF _bKpMode[_i] THEN
			sDeadTimeHelp[_i] := 'Kp tune: scoring this move. D is left unchanged.';
			sKpHelp[_i] := 'Scoring current Kp. Watch set vs actual; a suggestion appears when the move ends.';
		ELSE
			sDeadTimeHelp[_i] := 'SMC_EstimateDeadTime: waiting for constant velocity. Kp=0 gives the cleanest estimate.';
			sKpHelp[_i] := 'Kp is 0, so this pass estimates D only.';
		END_IF
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
	
	IF UseSuggestedKp[_i] THEN
		UseSuggestedKp[_i] := FALSE;
		IF bKpSuggested[_i] THEN
			PersistentVars.PositionControllers[_i].fKp := fSuggestedKp[_i];
			sKpHelp[_i] := 'Copied suggested Kp. Press WRITE PARAMETERS to apply.';
		ELSE
			sKpHelp[_i] := 'No suggested Kp yet. Set Kp>0, WRITE PARAMETERS, then START TUNE.';
		END_IF
	END_IF
	
	IF RaiseKp[_i] THEN
		RaiseKp[_i] := FALSE;
		_fKpNow := PersistentVars.PositionControllers[_i].fKp;
		IF _fKpNow < _fKpMin THEN
			_fKpNow := _fKpMin;
		END_IF
		fSuggestedKp[_i] := _fKpNow * _fKpRaise;
		IF fSuggestedKp[_i] > _fKpMax THEN
			fSuggestedKp[_i] := _fKpMax;
		END_IF
		PersistentVars.PositionControllers[_i].fKp := fSuggestedKp[_i];
		bKpSuggested[_i] := TRUE;
		sKpVerdict[_i] := 'RAISED';
		sKpHelp[_i] := 'Raised Kp to ';
		sKpHelp[_i] := CONCAT(sKpHelp[_i], TO_STRING(fSuggestedKp[_i]));
		sKpHelp[_i] := CONCAT(sKpHelp[_i], '. WRITE PARAMETERS, START TUNE, watch the graph. When it fluctuates, press LOWER.');
	END_IF
	
	IF LowerKp[_i] THEN
		LowerKp[_i] := FALSE;
		_fKpNow := PersistentVars.PositionControllers[_i].fKp;
		fSuggestedKp[_i] := _fKpNow * _fKpLower;
		IF fSuggestedKp[_i] < _fKpMin THEN
			fSuggestedKp[_i] := _fKpMin;
		END_IF
		PersistentVars.PositionControllers[_i].fKp := fSuggestedKp[_i];
		bKpSuggested[_i] := TRUE;
		sKpVerdict[_i] := 'LOWERED';
		sKpHelp[_i] := 'Lowered Kp to ';
		sKpHelp[_i] := CONCAT(sKpHelp[_i], TO_STRING(fSuggestedKp[_i]));
		sKpHelp[_i] := CONCAT(sKpHelp[_i], '. WRITE PARAMETERS, then START TUNE. Official finish is ~10% below the first fluctuation.');
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
		IF _bKpMode[_i] THEN
			_nKpSettle[_i] := _nKpSettleCycles;
			_bKpSettleInit[_i] := TRUE;
			sKpHelp[_i] := 'Move done, watching settle for a real bounce...';
		ELSIF _eDtState[_i] = 2 THEN
			_bDtFinish[_i] := TRUE;
		ELSIF _eDtState[_i] = 1 THEN
			_eDtState[_i] := 0;
			sDeadTimeHelp[_i] := 'Move ended before constant velocity. Increase MoveDistance or reduce acceleration.';
		END_IF
	END_IF

	CASE _i OF
		0:
			_fSetVel := IoConfig_Globals.J1.fSetVelocity;
			_fSetPos := IoConfig_Globals.J1.fSetPosition;
			_fActPos := IoConfig_Globals.J1.fActPosition;
		1:
			_fSetVel := IoConfig_Globals.J2.fSetVelocity;
			_fSetPos := IoConfig_Globals.J2.fSetPosition;
			_fActPos := IoConfig_Globals.J2.fActPosition;
		2:
			_fSetVel := IoConfig_Globals.J3.fSetVelocity;
			_fSetPos := IoConfig_Globals.J3.fSetPosition;
			_fActPos := IoConfig_Globals.J3.fActPosition;
		3:
			_fSetVel := IoConfig_Globals.J4.fSetVelocity;
			_fSetPos := IoConfig_Globals.J4.fSetPosition;
			_fActPos := IoConfig_Globals.J4.fActPosition;
		4:
			_fSetVel := IoConfig_Globals.J5.fSetVelocity;
			_fSetPos := IoConfig_Globals.J5.fSetPosition;
			_fActPos := IoConfig_Globals.J5.fActPosition;
		5:
			_fSetVel := IoConfig_Globals.J6.fSetVelocity;
			_fSetPos := IoConfig_Globals.J6.fSetPosition;
			_fActPos := IoConfig_Globals.J6.fActPosition;
	END_CASE
	
	_MoveBusyRtrig[_i](CLK:=MoveRelative[_i].Busy);
	IF _MoveBusyRtrig[_i].Q THEN
		IF _bKpMode[_i] THEN
			_bKpActive[_i] := TRUE;
			_bKpApproached[_i] := FALSE;
			_bKpFinish[_i] := FALSE;
			_nKpSettle[_i] := 0;
			_fKpDist[_i] := MoveRelative[_i].Distance;
			_fKpTarget[_i] := _fActPos + MoveRelative[_i].Distance;
			_fKpActStart[_i] := _fActPos;
			_fKpActMin[_i] := _fActPos;
			_fKpActMax[_i] := _fActPos;
			_fKpActLast[_i] := _fActPos;
			_fKpPrevTgtErr[_i] := 0.0;
			_fKpPrevActDelta[_i] := 0.0;
			_fKpCruiseSum[_i] := 0.0;
			_nKpCruiseN[_i] := 0;
			fKpPeakLag[_i] := 0.0;
			fKpCruiseLag[_i] := 0.0;
			fKpFinalLag[_i] := 0.0;
			fKpOvershoot[_i] := 0.0;
			fKpUndershoot[_i] := 0.0;
			nKpReversals[_i] := 0;
			nKpVelReversals[_i] := 0;
		ELSE
			_nSamples[_i] := 0;
			nDeadTimeSamples[_i] := 0;
			_bDtFinish[_i] := FALSE;
			_eDtState[_i] := 1;
		END_IF
	END_IF
	
	IF _bKpActive[_i] THEN
		_fLag := _fSetPos - _fActPos;
		IF ABS(_fLag) > fKpPeakLag[_i] THEN
			fKpPeakLag[_i] := ABS(_fLag);
		END_IF
		IF ABS(_fSetVel) >= (MoveRelative[_i].Velocity * _fCruiseRatio) THEN
			_fKpCruiseSum[_i] := _fKpCruiseSum[_i] + ABS(_fLag);
			_nKpCruiseN[_i] := _nKpCruiseN[_i] + 1;
		END_IF
		IF _fActPos < _fKpActMin[_i] THEN
			_fKpActMin[_i] := _fActPos;
		END_IF
		IF _fActPos > _fKpActMax[_i] THEN
			_fKpActMax[_i] := _fActPos;
		END_IF
		IF _bKpSettleInit[_i] THEN
			_bKpSettleInit[_i] := FALSE;
			_fKpActLast[_i] := _fActPos;
			_fKpSettleLeg[_i] := 0.0;
			nKpVelReversals[_i] := 0;
		ELSIF _nKpSettle[_i] > 0 THEN
			_fKpNow := _fActPos - _fKpActLast[_i];
			_fKpActLast[_i] := _fActPos;
			IF ABS(_fKpNow) > _fKpDeltaMin THEN
				IF (_fKpSettleLeg[_i] = 0.0) OR ((_fKpSettleLeg[_i] > 0.0) = (_fKpNow > 0.0)) THEN
					_fKpSettleLeg[_i] := _fKpSettleLeg[_i] + _fKpNow;
				ELSE
					IF ABS(_fKpSettleLeg[_i]) > _fKpBounceMin THEN
						nKpVelReversals[_i] := nKpVelReversals[_i] + 1;
					END_IF
					_fKpSettleLeg[_i] := _fKpNow;
				END_IF
			END_IF
		END_IF
		IF _nKpSettle[_i] > 0 THEN
			_nKpSettle[_i] := _nKpSettle[_i] - 1;
			IF _nKpSettle[_i] = 0 THEN
				_bKpFinish[_i] := TRUE;
			END_IF
		END_IF
	END_IF
	
	IF _bKpFinish[_i] THEN
		_bKpFinish[_i] := FALSE;
		_bKpActive[_i] := FALSE;
		_nKpSettle[_i] := 0;
		fKpFinalLag[_i] := ABS(_fKpTarget[_i] - _fActPos);
		IF _nKpCruiseN[_i] > 0 THEN
			fKpCruiseLag[_i] := _fKpCruiseSum[_i] / INT_TO_LREAL(_nKpCruiseN[_i]);
		ELSE
			fKpCruiseLag[_i] := fKpPeakLag[_i];
		END_IF
		IF _fKpDist[_i] >= 0.0 THEN
			fKpOvershoot[_i] := _fKpActMax[_i] - _fKpTarget[_i];
			fKpUndershoot[_i] := _fKpTarget[_i] - _fKpActMax[_i];
		ELSE
			fKpOvershoot[_i] := _fKpTarget[_i] - _fKpActMin[_i];
			fKpUndershoot[_i] := _fKpActMin[_i] - _fKpTarget[_i];
		END_IF
		IF fKpOvershoot[_i] < 0.0 THEN
			fKpOvershoot[_i] := 0.0;
		END_IF
		IF fKpUndershoot[_i] < 0.0 THEN
			fKpUndershoot[_i] := 0.0;
		END_IF
		_fKpNow := PersistentVars.PositionControllers[_i].fKp;
		IF nKpVelReversals[_i] >= 1 THEN
			sKpVerdict[_i] := 'TOO HOT';
			fSuggestedKp[_i] := _fKpNow * _fKpLower;
		ELSIF fKpUndershoot[_i] > _fKpReachBand THEN
			sKpVerdict[_i] := 'TOO SOFT';
			fSuggestedKp[_i] := _fKpNow * _fKpRaise;
		ELSE
			sKpVerdict[_i] := 'GOOD';
			fSuggestedKp[_i] := _fKpNow;
		END_IF
		IF fSuggestedKp[_i] < _fKpMin THEN
			fSuggestedKp[_i] := _fKpMin;
		ELSIF fSuggestedKp[_i] > _fKpMax THEN
			fSuggestedKp[_i] := _fKpMax;
		END_IF
		bKpSuggested[_i] := TRUE;
		sKpHelp[_i] := CONCAT(sKpVerdict[_i], '  und ');
		sKpHelp[_i] := CONCAT(sKpHelp[_i], TO_STRING(fKpUndershoot[_i]));
		sKpHelp[_i] := CONCAT(sKpHelp[_i], '  ov ');
		sKpHelp[_i] := CONCAT(sKpHelp[_i], TO_STRING(fKpOvershoot[_i]));
		sKpHelp[_i] := CONCAT(sKpHelp[_i], '  bounce ');
		sKpHelp[_i] := CONCAT(sKpHelp[_i], INT_TO_STRING(nKpVelReversals[_i]));
		sKpHelp[_i] := CONCAT(sKpHelp[_i], '  try Kp ');
		sKpHelp[_i] := CONCAT(sKpHelp[_i], TO_STRING(fSuggestedKp[_i]));
		sKpHelp[_i] := CONCAT(sKpHelp[_i], '. GOOD means this move did not bounce or miss. Use RAISE/LOWER and the graph.');
	END_IF
	
	_fCruiseVel := MoveRelative[_i].Velocity * _fCruiseRatio;
	IF (NOT _bKpMode[_i]) AND (_eDtState[_i] = 1) THEN
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

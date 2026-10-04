METHOD PRIVATE _M_Init
VAR_INPUT
END_VAR
VAR_STAT
	_Step: INT;
	_Done: BOOL;
	_Success: BOOL;
	_SMC_SetPosControlParams: ARRAY[0..5] OF SMC_SetPosControlParams;
	_i, _j, _k: INT;
END_VAR

CASE _Step OF
	0:
		IF InitializeDrives THEN
			InitializeDrives := FALSE;
			//
			FOR _i:=GVL_App.POSCONT_ARR_BEG TO GVL_App.POSCONT_ARR_END BY 1 DO
				_SMC_SetPosControlParams[_i].bExecute := FALSE;
			END_FOR
			//
			IF NOT PersistentVars.Flags[GVL_App.INIT_DRV_IDX].Value THEN
				FOR _i:=GVL_App.POSCONT_ARR_BEG TO GVL_App.POSCONT_ARR_END BY 1 DO
					PersistentVars.PositionControllers[_i].fKp := 0.1;
					PersistentVars.PositionControllers[_i].fPartVelPilotControl := 1.0; // 0-1
					PersistentVars.PositionControllers[_i].fDeadTime := 2.0;
					PersistentVars.PositionControllers[_i].fMaxPositionDiff := 0.0;
					PersistentVars.PositionControllers[_i].fSWMaxVelocity := 30.0;
					PersistentVars.PositionControllers[_i].fSWMaxAcceleration := 1000.0;
					PersistentVars.PositionControllers[_i].fSWMaxDeceleration := 1000.0;
					PersistentVars.PositionControllers[_i].fSWMaxJerk := 10000.0;
				END_FOR
			END_IF
			// next, please
			_Step := _Step + 1;
		END_IF
		
	1: 
		//
		FOR _i:=GVL_App.POSCONT_ARR_BEG TO GVL_App.POSCONT_ARR_END BY 1 DO
			_SMC_SetPosControlParams[_i].bExecute := TRUE;
		END_FOR
		// next, please
		_Step := _Step + 1;
		
	2:
		//
		_Done := TRUE;
		_Success := TRUE;
		FOR _i:=GVL_App.POSCONT_ARR_BEG TO GVL_App.POSCONT_ARR_END BY 1 DO
			IF _SMC_SetPosControlParams[_i].bError THEN
				_Success := FALSE;
				EXIT;
			END_IF
			IF NOT _SMC_SetPosControlParams[_i].bDone THEN
				_Success := FALSE;
				_Done := FALSE;
				EXIT;
			END_IF
		END_FOR
		IF _Done THEN
			// next, please
			_Step := _Step + 1;
		END_IF
		
	3:
		//
		FOR _i:=GVL_App.POSCONT_ARR_BEG TO GVL_App.POSCONT_ARR_END BY 1 DO
			_SMC_SetPosControlParams[_i].bExecute := FALSE;
		END_FOR
		//
		IF _Success THEN
			PersistentVars.Flags[GVL_App.INIT_DRV_IDX].Value := TRUE;
		END_IF
		// next, please
		_Step := 0;
		
END_CASE

//
_SMC_SetPosControlParams[0](
	Axis:= IoConfig_Globals.J1, 
	bExecute:= , 
	fKp:= PersistentVars.PositionControllers[0].fKp, 
	fPartVelPilotControl:= PersistentVars.PositionControllers[0].fPartVelPilotControl, 
	fDeadTime:= PersistentVars.PositionControllers[0].fDeadTime, 
	fMaxPositionDiff:=PersistentVars.PositionControllers[0].fMaxPositionDiff, 
	bDone=> , 
	bError=> , 
	ErrorID=> );
IF _SMC_SetPosControlParams[0].bDone AND NOT _SMC_SetPosControlParams[0].bError THEN
	IoConfig_Globals.J1.fSWMaxVelocity 			:= PersistentVars.PositionControllers[0].fSWMaxVelocity; // 30
	IoConfig_Globals.J1.fSWMaxAcceleration 		:= PersistentVars.PositionControllers[0].fSWMaxAcceleration; // 1000
	IoConfig_Globals.J1.fSWMaxDeceleration 		:= PersistentVars.PositionControllers[0].fSWMaxDeceleration; // 1000
	IoConfig_Globals.J1.fSWMaxJerk 				:= PersistentVars.PositionControllers[0].fSWMaxJerk; // 10000
END_IF

//
_SMC_SetPosControlParams[1](
	Axis:= IoConfig_Globals.J2, 
	bExecute:= , 
	fKp:= PersistentVars.PositionControllers[1].fKp, 
	fPartVelPilotControl:= PersistentVars.PositionControllers[1].fPartVelPilotControl, 
	fDeadTime:= PersistentVars.PositionControllers[1].fDeadTime, 
	fMaxPositionDiff:=PersistentVars.PositionControllers[1].fMaxPositionDiff, 
	bDone=> , 
	bError=> , 
	ErrorID=> );
IF _SMC_SetPosControlParams[1].bDone AND NOT _SMC_SetPosControlParams[1].bError THEN
	IoConfig_Globals.J2.fSWMaxVelocity 			:= PersistentVars.PositionControllers[1].fSWMaxVelocity;
	IoConfig_Globals.J2.fSWMaxAcceleration 		:= PersistentVars.PositionControllers[1].fSWMaxAcceleration;
	IoConfig_Globals.J2.fSWMaxDeceleration 		:= PersistentVars.PositionControllers[1].fSWMaxDeceleration;
	IoConfig_Globals.J2.fSWMaxJerk 				:= PersistentVars.PositionControllers[1].fSWMaxJerk;
END_IF

//
_SMC_SetPosControlParams[2](
	Axis:= IoConfig_Globals.J3, 
	bExecute:= , 
	fKp:= PersistentVars.PositionControllers[2].fKp, 
	fPartVelPilotControl:= PersistentVars.PositionControllers[2].fPartVelPilotControl, 
	fDeadTime:= PersistentVars.PositionControllers[2].fDeadTime, 
	fMaxPositionDiff:=PersistentVars.PositionControllers[2].fMaxPositionDiff, 
	bDone=> , 
	bError=> , 
	ErrorID=> );
IF _SMC_SetPosControlParams[2].bDone AND NOT _SMC_SetPosControlParams[2].bError THEN
	IoConfig_Globals.J3.fSWMaxVelocity 			:= PersistentVars.PositionControllers[2].fSWMaxVelocity;
	IoConfig_Globals.J3.fSWMaxAcceleration 		:= PersistentVars.PositionControllers[2].fSWMaxAcceleration;
	IoConfig_Globals.J3.fSWMaxDeceleration 		:= PersistentVars.PositionControllers[2].fSWMaxDeceleration;
	IoConfig_Globals.J3.fSWMaxJerk 				:= PersistentVars.PositionControllers[2].fSWMaxJerk;
END_IF

//
_SMC_SetPosControlParams[3](
	Axis:= IoConfig_Globals.J4, 
	bExecute:= , 
	fKp:= PersistentVars.PositionControllers[3].fKp, 
	fPartVelPilotControl:= PersistentVars.PositionControllers[3].fPartVelPilotControl, 
	fDeadTime:= PersistentVars.PositionControllers[3].fDeadTime, 
	fMaxPositionDiff:=PersistentVars.PositionControllers[3].fMaxPositionDiff, 
	bDone=> , 
	bError=> , 
	ErrorID=> );
IF _SMC_SetPosControlParams[3].bDone AND NOT _SMC_SetPosControlParams[3].bError THEN
	IoConfig_Globals.J4.fSWMaxVelocity 			:= PersistentVars.PositionControllers[3].fSWMaxVelocity;
	IoConfig_Globals.J4.fSWMaxAcceleration 		:= PersistentVars.PositionControllers[3].fSWMaxAcceleration;
	IoConfig_Globals.J4.fSWMaxDeceleration 		:= PersistentVars.PositionControllers[3].fSWMaxDeceleration;
	IoConfig_Globals.J4.fSWMaxJerk 				:= PersistentVars.PositionControllers[3].fSWMaxJerk;
END_IF

//
_SMC_SetPosControlParams[4](
	Axis:= IoConfig_Globals.J5, 
	bExecute:= , 
	fKp:= PersistentVars.PositionControllers[4].fKp, 
	fPartVelPilotControl:= PersistentVars.PositionControllers[4].fPartVelPilotControl, 
	fDeadTime:= PersistentVars.PositionControllers[4].fDeadTime, 
	fMaxPositionDiff:=PersistentVars.PositionControllers[4].fMaxPositionDiff, 
	bDone=> , 
	bError=> , 
	ErrorID=> );
IF _SMC_SetPosControlParams[4].bDone AND NOT _SMC_SetPosControlParams[4].bError THEN
	IoConfig_Globals.J5.fSWMaxVelocity 			:= PersistentVars.PositionControllers[4].fSWMaxVelocity;
	IoConfig_Globals.J5.fSWMaxAcceleration 		:= PersistentVars.PositionControllers[4].fSWMaxAcceleration;
	IoConfig_Globals.J5.fSWMaxDeceleration 		:= PersistentVars.PositionControllers[4].fSWMaxDeceleration;
	IoConfig_Globals.J5.fSWMaxJerk 				:= PersistentVars.PositionControllers[4].fSWMaxJerk;
END_IF

//
_SMC_SetPosControlParams[5](
	Axis:= IoConfig_Globals.J6, 
	bExecute:= , 
	fKp:= PersistentVars.PositionControllers[5].fKp, 
	fPartVelPilotControl:= PersistentVars.PositionControllers[5].fPartVelPilotControl, 
	fDeadTime:= PersistentVars.PositionControllers[5].fDeadTime, 
	fMaxPositionDiff:=PersistentVars.PositionControllers[5].fMaxPositionDiff, 
	bDone=> , 
	bError=> , 
	ErrorID=> );
IF _SMC_SetPosControlParams[5].bDone AND NOT _SMC_SetPosControlParams[5].bError THEN
	IoConfig_Globals.J6.fSWMaxVelocity 			:= PersistentVars.PositionControllers[5].fSWMaxVelocity;
	IoConfig_Globals.J6.fSWMaxAcceleration 		:= PersistentVars.PositionControllers[5].fSWMaxAcceleration;
	IoConfig_Globals.J6.fSWMaxDeceleration 		:= PersistentVars.PositionControllers[5].fSWMaxDeceleration;
	IoConfig_Globals.J6.fSWMaxJerk 				:= PersistentVars.PositionControllers[5].fSWMaxJerk;
END_IF
END_METHOD

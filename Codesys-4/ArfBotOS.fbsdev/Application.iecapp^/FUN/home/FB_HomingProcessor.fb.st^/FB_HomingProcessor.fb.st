FUNCTION_BLOCK FB_HomingProcessor
VAR_IN_OUT
	J1, J2, J3, J4, J5, J6: AXIS_REF_SM3;
	AxisGroup: AXIS_GROUP_REF_SM3;
	GroupHandler: FB_GroupHandler;
	Controller: FB_Controller;
END_VAR

VAR_INPUT
	J1_Limit, J2_Limit, J3_Limit, J4_Limit, J5_Limit, J6_Limit: BOOL;
	Enable: BOOL;
	Reset: BOOL;
	StoredPositions: REFERENCE TO ARRAY[GVL_App.POS_ARR_BEG..GVL_App.POS_ARR_END] OF DUT_StoredPosition;
	VisionRegisters: REFERENCE TO ARRAY[GVL_App.VIS_ARR_BEG..GVL_App.VIS_ARR_END] OF DUT_VisionRegister;
	Flags: REFERENCE TO ARRAY[GVL_App.FLG_ARR_BEG..GVL_App.FLG_ARR_END] OF DUT_Flags;
END_VAR
VAR_OUTPUT
	Done: BOOL;
	Busy: BOOL;
	Error: BOOL;
	ErrorID: SMC_ERROR;
END_VAR
VAR
	_ExecuteRtrig: R_TRIG;
	_ExecuteFtrig: F_TRIG;
	_ResetRtrig: R_TRIG;
	_ErrorRtrig: R_TRIG;
	
	_SequenceMain: INT;
	_StartHoming: BOOL;
	
	_SMC_Homing_J1, _SMC_Homing_J2, _SMC_Homing_J3: SMC_Homing;
	_SMC_Homing_J4, _SMC_Homing_J5, _SMC_Homing_J6: SMC_Homing;
	
	_MC_SetPosition_J1, _MC_SetPosition_J2, _MC_SetPosition_J3: MC_SetPosition;
	_MC_SetPosition_J4, _MC_SetPosition_J5, _MC_SetPosition_J6: MC_SetPosition;
	
	_HomingVelocityFast: LREAL:= 15.0;
	_HomingVelocitySlow: LREAL:= 10.0;
	_HomingVelocitySlowSlow: LREAL:= 1.0;
	_HomingAcceleration: LREAL:= 50.0;
	_HomingJerk: LREAL:= 100.0;
	
	_MoveToPose: BOOL;
	_MoveCommand: ST_MoveCmd;
	_MoveCommandProcessor: FB_MoveCmdProcessor;
	
	_i, _j, _k: INT;
	_FirstScan: BOOL:=TRUE;
END_VAR

VAR CONSTANT
	STEP_DONE: 					INT := 9999;
	STEP_IDLE: 					INT := 0000;
	STEP_GROUP_ENABLE: 			INT := 0030;
	STEP_WAIT_ENABLE: 			INT := 0035;
	STEP_FAST_HOME_G1:			INT := 0040;
	STEP_WAIT_FAST_HOME_G1:		INT := 0045;
	STEP_SLOW_HOME_G1:			INT := 0050;
	STEP_WAIT_SLOW_HOME_G1:		INT := 0055;
	
	STEP_FAST_HOME_G2:			INT := 0060;
	STEP_WAIT_FAST_HOME_G2:		INT := 0065;
	STEP_SLOW_HOME_G2:			INT := 0070;
	STEP_WAIT_SLOW_HOME_G2:		INT := 0075;
	
	STEP_FAST_HOME_G3:			INT := 0080;
	STEP_WAIT_FAST_HOME_G3:		INT := 0085;
	STEP_SLOW_HOME_G3:			INT := 0090;
	STEP_WAIT_SLOW_HOME_G3:		INT := 0095;
	
	STEP_MOVE_TO_POSE:			INT := 1000;
	STEP_WAIT_MOVE_TO_POSE:		INT := 1010;
	STEP_WAIT_POSE_DONE:		INT := 1020;
END_VAR

// TODO refactor all the code in the FB to use arrays and for loops ... whenever you get bored...

_ExecuteRtrig(CLK:=_StartHoming);
_ExecuteFtrig(CLK:=_StartHoming);
_ResetRtrig(CLK:=Reset);

IF _ExecuteRtrig.Q THEN
    _StartHoming := FALSE;
END_IF

IF _FirstScan THEN
	//
END_IF

CASE _SequenceMain OF
	STEP_DONE: // done
		Busy := FALSE;
		Done := TRUE;
		_MoveCommandProcessor.M_Home();
		_SequenceMain := STEP_IDLE;
		
		_SMC_Homing_J1.bExecute := FALSE;
		_SMC_Homing_J2.bExecute := FALSE;
		_SMC_Homing_J3.bExecute := FALSE;
		_SMC_Homing_J4.bExecute := FALSE;
		_SMC_Homing_J5.bExecute := FALSE;
		_SMC_Homing_J6.bExecute := FALSE;
		
		J1.bSWLimitEnable := TRUE;
		J2.bSWLimitEnable := TRUE;
		J3.bSWLimitEnable := TRUE;
		J4.bSWLimitEnable := TRUE;
		J5.bSWLimitEnable := TRUE;
		J6.bSWLimitEnable := TRUE;
		
	STEP_IDLE: // IDLE
		IF _ExecuteRtrig.Q AND Enable THEN
			Busy := TRUE;
			Done := FALSE;
			_SequenceMain := STEP_GROUP_ENABLE;
			Flags[GVL_App.HOME_COMPLETE].Value := FALSE;
		END_IF
		
	STEP_GROUP_ENABLE:
		IF GroupHandler.M_EnableGroup() THEN
			_SequenceMain := STEP_WAIT_ENABLE;
		ELSE
			_SequenceMain := STEP_DONE;
			Error := TRUE;
			ErrorID := GroupHandler.ErrorID;
		END_IF
		
	STEP_WAIT_ENABLE:
		// TODO add timeout timer here
		IF GroupHandler.P_GroupEnabled THEN
			_SequenceMain := STEP_FAST_HOME_G1;
		END_IF;
		
	STEP_FAST_HOME_G1:	
		_SMC_Homing_J1.bExecute := SEL(PersistentVars.HomingGroupOne.J1, FALSE, TRUE);
		_SMC_Homing_J2.bExecute := SEL(PersistentVars.HomingGroupOne.J2, FALSE, TRUE);
		_SMC_Homing_J3.bExecute := SEL(PersistentVars.HomingGroupOne.J3, FALSE, TRUE);
		_SMC_Homing_J4.bExecute := SEL(PersistentVars.HomingGroupOne.J4, FALSE, TRUE);
		_SMC_Homing_J5.bExecute := SEL(PersistentVars.HomingGroupOne.J5, FALSE, TRUE);
		_SMC_Homing_J6.bExecute := SEL(PersistentVars.HomingGroupOne.J6, FALSE, TRUE);
		
		_SMC_Homing_J1.bReturnToZero := FALSE;
		_SMC_Homing_J2.bReturnToZero := FALSE;
		_SMC_Homing_J3.bReturnToZero := FALSE;
		_SMC_Homing_J4.bReturnToZero := FALSE;
		_SMC_Homing_J5.bReturnToZero := FALSE;
		_SMC_Homing_J6.bReturnToZero := FALSE;
		
		_SMC_Homing_J1.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J2.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J3.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J4.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J5.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J6.fVelocitySlow := _HomingVelocitySlow;
		
		J1.bSWLimitEnable := FALSE;
		J2.bSWLimitEnable := FALSE;
		J3.bSWLimitEnable := FALSE;
		J4.bSWLimitEnable := FALSE;
		J5.bSWLimitEnable := FALSE;
		J6.bSWLimitEnable := FALSE;
		
		_SequenceMain := STEP_WAIT_FAST_HOME_G1;
		
	STEP_WAIT_FAST_HOME_G1: // wait fast home group 1
		IF SEL(PersistentVars.HomingGroupOne.J1, TRUE, _SMC_Homing_J1.bDone)
			AND SEL(PersistentVars.HomingGroupOne.J2, TRUE, _SMC_Homing_J2.bDone)
			AND SEL(PersistentVars.HomingGroupOne.J3, TRUE, _SMC_Homing_J3.bDone) 
			AND SEL(PersistentVars.HomingGroupOne.J4, TRUE, _SMC_Homing_J4.bDone) 
			AND SEL(PersistentVars.HomingGroupOne.J5, TRUE, _SMC_Homing_J5.bDone) 
			AND SEL(PersistentVars.HomingGroupOne.J6, TRUE, _SMC_Homing_J6.bDone) THEN
				_SMC_Homing_J1.bExecute := FALSE;
				_SMC_Homing_J2.bExecute := FALSE;
				_SMC_Homing_J3.bExecute := FALSE;
				_SMC_Homing_J4.bExecute := FALSE;
				_SMC_Homing_J5.bExecute := FALSE;
				_SMC_Homing_J6.bExecute := FALSE;
				
				_SequenceMain := STEP_SLOW_HOME_G1;
		END_IF
		
	STEP_SLOW_HOME_G1: // slow home group 1
		_SMC_Homing_J1.bExecute := SEL(PersistentVars.HomingGroupOne.J1, FALSE, TRUE);
		_SMC_Homing_J2.bExecute := SEL(PersistentVars.HomingGroupOne.J2, FALSE, TRUE);
		_SMC_Homing_J3.bExecute := SEL(PersistentVars.HomingGroupOne.J3, FALSE, TRUE);
		_SMC_Homing_J4.bExecute := SEL(PersistentVars.HomingGroupOne.J4, FALSE, TRUE);
		_SMC_Homing_J5.bExecute := SEL(PersistentVars.HomingGroupOne.J5, FALSE, TRUE);
		_SMC_Homing_J6.bExecute := SEL(PersistentVars.HomingGroupOne.J6, FALSE, TRUE);
		
		_SMC_Homing_J1.bReturnToZero := TRUE;
		_SMC_Homing_J2.bReturnToZero := TRUE;
		_SMC_Homing_J3.bReturnToZero := TRUE;
		_SMC_Homing_J4.bReturnToZero := TRUE;
		_SMC_Homing_J5.bReturnToZero := TRUE;
		_SMC_Homing_J6.bReturnToZero := TRUE;
		
		_SMC_Homing_J1.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J2.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J3.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J4.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J5.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J6.fVelocitySlow := _HomingVelocitySlowSlow;
		
		_SequenceMain := STEP_WAIT_SLOW_HOME_G1;
		
	STEP_WAIT_SLOW_HOME_G1:
		IF SEL(PersistentVars.HomingGroupOne.J1, TRUE, _SMC_Homing_J1.bDone)
			AND SEL(PersistentVars.HomingGroupOne.J2, TRUE, _SMC_Homing_J2.bDone)
			AND SEL(PersistentVars.HomingGroupOne.J3, TRUE, _SMC_Homing_J3.bDone) 
			AND SEL(PersistentVars.HomingGroupOne.J4, TRUE, _SMC_Homing_J4.bDone) 
			AND SEL(PersistentVars.HomingGroupOne.J5, TRUE, _SMC_Homing_J5.bDone) 
			AND SEL(PersistentVars.HomingGroupOne.J6, TRUE, _SMC_Homing_J6.bDone) THEN
				_SMC_Homing_J1.bExecute := FALSE;
				_SMC_Homing_J2.bExecute := FALSE;
				_SMC_Homing_J3.bExecute := FALSE;
				_SMC_Homing_J4.bExecute := FALSE;
				_SMC_Homing_J5.bExecute := FALSE;
				_SMC_Homing_J6.bExecute := FALSE;
				
				_SequenceMain := STEP_FAST_HOME_G2;
		END_IF
		
	STEP_FAST_HOME_G2:
		_SMC_Homing_J1.bExecute := SEL(PersistentVars.HomingGroupTwo.J1, FALSE, TRUE);
		_SMC_Homing_J2.bExecute := SEL(PersistentVars.HomingGroupTwo.J2, FALSE, TRUE);
		_SMC_Homing_J3.bExecute := SEL(PersistentVars.HomingGroupTwo.J3, FALSE, TRUE);
		_SMC_Homing_J4.bExecute := SEL(PersistentVars.HomingGroupTwo.J4, FALSE, TRUE);
		_SMC_Homing_J5.bExecute := SEL(PersistentVars.HomingGroupTwo.J5, FALSE, TRUE);
		_SMC_Homing_J6.bExecute := SEL(PersistentVars.HomingGroupTwo.J6, FALSE, TRUE);
		
		_SMC_Homing_J1.bReturnToZero := FALSE;
		_SMC_Homing_J2.bReturnToZero := FALSE;
		_SMC_Homing_J3.bReturnToZero := FALSE;
		_SMC_Homing_J4.bReturnToZero := FALSE;
		_SMC_Homing_J5.bReturnToZero := FALSE;
		_SMC_Homing_J6.bReturnToZero := FALSE;
		
		_SMC_Homing_J1.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J2.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J3.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J4.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J5.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J6.fVelocitySlow := _HomingVelocitySlow;

		_SequenceMain := STEP_WAIT_FAST_HOME_G2;
		
	STEP_WAIT_FAST_HOME_G2:
		IF SEL(PersistentVars.HomingGroupTwo.J1, TRUE, _SMC_Homing_J1.bDone)
			AND SEL(PersistentVars.HomingGroupTwo.J2, TRUE, _SMC_Homing_J2.bDone)
			AND SEL(PersistentVars.HomingGroupTwo.J3, TRUE, _SMC_Homing_J3.bDone) 
			AND SEL(PersistentVars.HomingGroupTwo.J4, TRUE, _SMC_Homing_J4.bDone) 
			AND SEL(PersistentVars.HomingGroupTwo.J5, TRUE, _SMC_Homing_J5.bDone) 
			AND SEL(PersistentVars.HomingGroupTwo.J6, TRUE, _SMC_Homing_J6.bDone) THEN
				_SMC_Homing_J1.bExecute := FALSE;
				_SMC_Homing_J2.bExecute := FALSE;
				_SMC_Homing_J3.bExecute := FALSE;
				_SMC_Homing_J4.bExecute := FALSE;
				_SMC_Homing_J5.bExecute := FALSE;
				_SMC_Homing_J6.bExecute := FALSE;
				
				_SequenceMain := STEP_SLOW_HOME_G2;
		END_IF
		
	STEP_SLOW_HOME_G2:
		_SMC_Homing_J1.bExecute := SEL(PersistentVars.HomingGroupTwo.J1, FALSE, TRUE);
		_SMC_Homing_J2.bExecute := SEL(PersistentVars.HomingGroupTwo.J2, FALSE, TRUE);
		_SMC_Homing_J3.bExecute := SEL(PersistentVars.HomingGroupTwo.J3, FALSE, TRUE);
		_SMC_Homing_J4.bExecute := SEL(PersistentVars.HomingGroupTwo.J4, FALSE, TRUE);
		_SMC_Homing_J5.bExecute := SEL(PersistentVars.HomingGroupTwo.J5, FALSE, TRUE);
		_SMC_Homing_J6.bExecute := SEL(PersistentVars.HomingGroupTwo.J6, FALSE, TRUE);
		
		_SMC_Homing_J1.bReturnToZero := TRUE;
		_SMC_Homing_J2.bReturnToZero := TRUE;
		_SMC_Homing_J3.bReturnToZero := TRUE;
		_SMC_Homing_J4.bReturnToZero := TRUE;
		_SMC_Homing_J5.bReturnToZero := TRUE;
		_SMC_Homing_J6.bReturnToZero := TRUE;
		
		_SMC_Homing_J1.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J2.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J3.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J4.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J5.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J6.fVelocitySlow := _HomingVelocitySlowSlow;
		
		_SequenceMain := STEP_WAIT_SLOW_HOME_G2;
		
	STEP_WAIT_SLOW_HOME_G2:
		IF SEL(PersistentVars.HomingGroupTwo.J1, TRUE, _SMC_Homing_J1.bDone)
			AND SEL(PersistentVars.HomingGroupTwo.J2, TRUE, _SMC_Homing_J2.bDone)
			AND SEL(PersistentVars.HomingGroupTwo.J3, TRUE, _SMC_Homing_J3.bDone) 
			AND SEL(PersistentVars.HomingGroupTwo.J4, TRUE, _SMC_Homing_J4.bDone) 
			AND SEL(PersistentVars.HomingGroupTwo.J5, TRUE, _SMC_Homing_J5.bDone) 
			AND SEL(PersistentVars.HomingGroupTwo.J6, TRUE, _SMC_Homing_J6.bDone) THEN
				_SMC_Homing_J1.bExecute := FALSE;
				_SMC_Homing_J2.bExecute := FALSE;
				_SMC_Homing_J3.bExecute := FALSE;
				_SMC_Homing_J4.bExecute := FALSE;
				_SMC_Homing_J5.bExecute := FALSE;
				_SMC_Homing_J6.bExecute := FALSE;
				
				_SequenceMain := STEP_FAST_HOME_G3;
		END_IF
		
	STEP_FAST_HOME_G3:
		_SMC_Homing_J1.bExecute := SEL(PersistentVars.HomingGroupThree.J1, FALSE, TRUE);
		_SMC_Homing_J2.bExecute := SEL(PersistentVars.HomingGroupThree.J2, FALSE, TRUE);
		_SMC_Homing_J3.bExecute := SEL(PersistentVars.HomingGroupThree.J3, FALSE, TRUE);
		_SMC_Homing_J4.bExecute := SEL(PersistentVars.HomingGroupThree.J4, FALSE, TRUE);
		_SMC_Homing_J5.bExecute := SEL(PersistentVars.HomingGroupThree.J5, FALSE, TRUE);
		_SMC_Homing_J6.bExecute := SEL(PersistentVars.HomingGroupThree.J6, FALSE, TRUE);
		
		_SMC_Homing_J1.bReturnToZero := FALSE;
		_SMC_Homing_J2.bReturnToZero := FALSE;
		_SMC_Homing_J3.bReturnToZero := FALSE;
		_SMC_Homing_J4.bReturnToZero := FALSE;
		_SMC_Homing_J5.bReturnToZero := FALSE;
		_SMC_Homing_J6.bReturnToZero := FALSE;
		
		_SMC_Homing_J1.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J2.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J3.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J4.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J5.fVelocitySlow := _HomingVelocitySlow;
		_SMC_Homing_J6.fVelocitySlow := _HomingVelocitySlow;
		
		_SequenceMain := STEP_WAIT_FAST_HOME_G3;
		
	STEP_WAIT_FAST_HOME_G3:
		IF SEL(PersistentVars.HomingGroupThree.J1, TRUE, _SMC_Homing_J1.bDone)
			AND SEL(PersistentVars.HomingGroupThree.J2, TRUE, _SMC_Homing_J2.bDone)
			AND SEL(PersistentVars.HomingGroupThree.J3, TRUE, _SMC_Homing_J3.bDone) 
			AND SEL(PersistentVars.HomingGroupThree.J4, TRUE, _SMC_Homing_J4.bDone) 
			AND SEL(PersistentVars.HomingGroupThree.J5, TRUE, _SMC_Homing_J5.bDone) 
			AND SEL(PersistentVars.HomingGroupThree.J6, TRUE, _SMC_Homing_J6.bDone) THEN
				_SMC_Homing_J1.bExecute := FALSE;
				_SMC_Homing_J2.bExecute := FALSE;
				_SMC_Homing_J3.bExecute := FALSE;
				_SMC_Homing_J4.bExecute := FALSE;
				_SMC_Homing_J5.bExecute := FALSE;
				_SMC_Homing_J6.bExecute := FALSE;
				
				_SequenceMain := STEP_SLOW_HOME_G3;
		END_IF
		
	STEP_SLOW_HOME_G3:
		_SMC_Homing_J1.bExecute := SEL(PersistentVars.HomingGroupThree.J1, FALSE, TRUE);
		_SMC_Homing_J2.bExecute := SEL(PersistentVars.HomingGroupThree.J2, FALSE, TRUE);
		_SMC_Homing_J3.bExecute := SEL(PersistentVars.HomingGroupThree.J3, FALSE, TRUE);
		_SMC_Homing_J4.bExecute := SEL(PersistentVars.HomingGroupThree.J4, FALSE, TRUE);
		_SMC_Homing_J5.bExecute := SEL(PersistentVars.HomingGroupThree.J5, FALSE, TRUE);
		_SMC_Homing_J6.bExecute := SEL(PersistentVars.HomingGroupThree.J6, FALSE, TRUE);
		
		_SMC_Homing_J1.bReturnToZero := TRUE;
		_SMC_Homing_J2.bReturnToZero := TRUE;
		_SMC_Homing_J3.bReturnToZero := TRUE;
		_SMC_Homing_J4.bReturnToZero := TRUE;
		_SMC_Homing_J5.bReturnToZero := TRUE;
		_SMC_Homing_J6.bReturnToZero := TRUE;
		
		_SMC_Homing_J1.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J2.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J3.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J4.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J5.fVelocitySlow := _HomingVelocitySlowSlow;
		_SMC_Homing_J6.fVelocitySlow := _HomingVelocitySlowSlow;
		
		_SequenceMain := STEP_WAIT_SLOW_HOME_G3;
		
	STEP_WAIT_SLOW_HOME_G3:
		IF SEL(PersistentVars.HomingGroupThree.J1, TRUE, _SMC_Homing_J1.bDone)
			AND SEL(PersistentVars.HomingGroupThree.J2, TRUE, _SMC_Homing_J2.bDone)
			AND SEL(PersistentVars.HomingGroupThree.J3, TRUE, _SMC_Homing_J3.bDone) 
			AND SEL(PersistentVars.HomingGroupThree.J4, TRUE, _SMC_Homing_J4.bDone) 
			AND SEL(PersistentVars.HomingGroupThree.J5, TRUE, _SMC_Homing_J5.bDone) 
			AND SEL(PersistentVars.HomingGroupThree.J6, TRUE, _SMC_Homing_J6.bDone) THEN
				_SMC_Homing_J1.bExecute := FALSE;
				_SMC_Homing_J2.bExecute := FALSE;
				_SMC_Homing_J3.bExecute := FALSE;
				_SMC_Homing_J4.bExecute := FALSE;
				_SMC_Homing_J5.bExecute := FALSE;
				_SMC_Homing_J6.bExecute := FALSE;
				
				Flags[GVL_App.HOME_COMPLETE].Value := TRUE;
				IF NOT Flags[GVL_App.HOM_TO_POSE_IDX].Value THEN
					_SequenceMain := STEP_DONE;
				ELSE
					_SequenceMain := STEP_MOVE_TO_POSE;
				END_IF
		END_IF;
		
	STEP_MOVE_TO_POSE:
		_MoveToPose := TRUE;
		_SequenceMain := STEP_WAIT_MOVE_TO_POSE;
	
	STEP_WAIT_MOVE_TO_POSE:
		IF _MoveCommandProcessor.ItfUnitMode.CurrentState <> PACK_ML.State.Idle THEN
			_SequenceMain := STEP_WAIT_POSE_DONE;
		END_IF
		
	STEP_WAIT_POSE_DONE:
		IF _MoveCommandProcessor.ItfUnitMode.CurrentState = PACK_ML.State.Idle THEN
			_SequenceMain := STEP_DONE;
		END_IF
	
END_CASE

_M_HandlePose(AxisGroup:=AxisGroup, GroupHandler:=GroupHandler);

//
IF _ResetRtrig.Q AND Error THEN
	Error := FALSE;
	ErrorID := SMC_NO_ERROR;
	_SequenceMain := STEP_DONE;
	GroupHandler.M_ResetGroup();
	_MoveCommandProcessor.M_Home();
END_IF

//
_ErrorRtrig(CLK:=GroupHandler.Error
				OR _SMC_Homing_J1.bError
				OR _SMC_Homing_J2.bError
				OR _SMC_Homing_J3.bError
				OR _SMC_Homing_J4.bError
				OR _SMC_Homing_J5.bError
				OR _SMC_Homing_J6.bError
				OR _MoveCommandProcessor.Error);			
IF _ErrorRtrig.Q THEN
	Error := TRUE;
	IF GroupHandler.Error THEN
		ErrorID := GroupHandler.ErrorID;
	ELSIF _SMC_Homing_J1.bError THEN
		ErrorID := _SMC_Homing_J1.nErrorID;
	ELSIF _SMC_Homing_J2.bError THEN
		ErrorID := _SMC_Homing_J2.nErrorID;
	ELSIF _SMC_Homing_J3.bError THEN
		ErrorID := _SMC_Homing_J3.nErrorID;
	ELSIF _SMC_Homing_J4.bError THEN
		ErrorID := _SMC_Homing_J4.nErrorID;
	ELSIF _SMC_Homing_J5.bError THEN
		ErrorID := _SMC_Homing_J5.nErrorID;
	ELSIF _SMC_Homing_J6.bError THEN
		ErrorID := _SMC_Homing_J6.nErrorID;
	ELSIF _MoveCommandProcessor.Error THEN
		// TODO need to rework how error messages are handled (globally)
		// if the move command processor has an error, we dont know what it is.
	END_IF
END_IF

// TODO make search direction configurable for any robot
//
_SMC_Homing_J1(
	Axis:=J1 , 
	bExecute:= ,
	fHomePosition:= StoredPositions[GVL_App.HOM_POS_IDX].Position.X, 
	fVelocitySlow:= , 
	fVelocityFast:= _HomingVelocityFast, 
	fAcceleration:= _HomingAcceleration, 
	fDeceleration:= _HomingAcceleration, 
	fJerk:= _HomingJerk, 
	nDirection:= MC_direction.negative, 
	bReferenceSwitch:= NOT J1_Limit, 
	fSignalDelay:= , 
	nHomingMode:= , 
	bReturnToZero:= , 
	bIndexOccured:= , 
	fIndexPosition:= , 
	bIgnoreHWLimit:= TRUE, 
	bDone=> , 
	bBusy=> , 
	bCommandAborted=> , 
	bError=> , 
	nErrorID=> , 
	bStartLatchingIndex=> );
	
_SMC_Homing_J2(
	Axis:=J2 , 
	bExecute:= , 
	fHomePosition:= StoredPositions[GVL_App.HOM_POS_IDX].Position.Y,  
	fVelocitySlow:= , 
	fVelocityFast:= _HomingVelocityFast, 
	fAcceleration:= _HomingAcceleration, 
	fDeceleration:= _HomingAcceleration,
	fJerk:= _HomingJerk, 
	nDirection:= MC_Direction.negative, 
	bReferenceSwitch:= NOT J2_Limit, 
	fSignalDelay:= , 
	nHomingMode:= , 
	bReturnToZero:= , 
	bIndexOccured:= , 
	fIndexPosition:= , 
	bIgnoreHWLimit:= TRUE, 
	bDone=> , 
	bBusy=> , 
	bCommandAborted=> , 
	bError=> , 
	nErrorID=> , 
	bStartLatchingIndex=> );
	
_SMC_Homing_J3(
	Axis:=J3 , 
	bExecute:= , 
	fHomePosition:= StoredPositions[GVL_App.HOM_POS_IDX].Position.Z, 
	fVelocitySlow:= , 
	fVelocityFast:= _HomingVelocityFast, 
	fAcceleration:= _HomingAcceleration, 
	fDeceleration:= _HomingAcceleration,
	fJerk:= _HomingJerk, 
	nDirection:= MC_Direction.positive, 
	bReferenceSwitch:= NOT J3_Limit, 
	fSignalDelay:= , 
	nHomingMode:= , 
	bReturnToZero:= , 
	bIndexOccured:= , 
	fIndexPosition:= , 
	bIgnoreHWLimit:= TRUE, 
	bDone=> , 
	bBusy=> , 
	bCommandAborted=> , 
	bError=> , 
	nErrorID=> , 
	bStartLatchingIndex=> );
	
_SMC_Homing_J4(
	Axis:=J4 , 
	bExecute:= ,
	fHomePosition:= StoredPositions[GVL_App.HOM_POS_IDX].Position.A, 
	fVelocitySlow:= , 
	fVelocityFast:= _HomingVelocityFast, 
	fAcceleration:= _HomingAcceleration, 
	fDeceleration:= _HomingAcceleration,
	fJerk:= _HomingJerk, 
	nDirection:= MC_Direction.positive, 
	bReferenceSwitch:= NOT J4_Limit, 
	fSignalDelay:= , 
	nHomingMode:= , 
	bReturnToZero:= , 
	bIndexOccured:= , 
	fIndexPosition:= , 
	bIgnoreHWLimit:= TRUE, 
	bDone=> , 
	bBusy=> , 
	bCommandAborted=> , 
	bError=> , 
	nErrorID=> , 
	bStartLatchingIndex=> );
	
_SMC_Homing_J5(
	Axis:=J5 , 
	bExecute:= , 
	fHomePosition:= StoredPositions[GVL_App.HOM_POS_IDX].Position.B,  
	fVelocitySlow:= , 
	fVelocityFast:= _HomingVelocityFast, 
	fAcceleration:= _HomingAcceleration, 
	fDeceleration:= _HomingAcceleration,
	fJerk:= _HomingJerk, 
	nDirection:= MC_Direction.positive, 
	bReferenceSwitch:= NOT J5_Limit, 
	fSignalDelay:= , 
	nHomingMode:= , 
	bReturnToZero:= , 
	bIndexOccured:= , 
	fIndexPosition:= , 
	bIgnoreHWLimit:= TRUE, 
	bDone=> , 
	bBusy=> , 
	bCommandAborted=> , 
	bError=> , 
	nErrorID=> , 
	bStartLatchingIndex=> );
	
_SMC_Homing_J6(
	Axis:=J6 , 
	bExecute:= , 
	fHomePosition:= StoredPositions[GVL_App.HOM_POS_IDX].Position.C, 
	fVelocitySlow:= , 
	fVelocityFast:= _HomingVelocityFast, 
	fAcceleration:= _HomingAcceleration, 
	fDeceleration:= _HomingAcceleration,
	fJerk:= _HomingJerk, 
	nDirection:= MC_Direction.negative, 
	bReferenceSwitch:= NOT J6_Limit, 
	fSignalDelay:= , 
	nHomingMode:= , 
	bReturnToZero:= , 
	bIndexOccured:= , 
	fIndexPosition:= , 
	bIgnoreHWLimit:= TRUE, 
	bDone=> , 
	bBusy=> , 
	bCommandAborted=> , 
	bError=> , 
	nErrorID=> , 
	bStartLatchingIndex=> );
	
//
_MC_SetPosition_J1(
	Axis:= J1, 
	Execute:= _MC_SetPosition_J1.Execute AND NOT _MC_SetPosition_J1.Done AND NOT _MC_SetPosition_J1.Error, 
	Position:= 0.0, 
	Mode:= FALSE, // true=relative false=absolute
	Done=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );
_MC_SetPosition_J2(
	Axis:= J2, 
	Execute:= _MC_SetPosition_J2.Execute AND NOT _MC_SetPosition_J2.Done AND NOT _MC_SetPosition_J2.Error, 
	Position:= 0.0, 
	Mode:= FALSE, // true=relative false=absolute
	Done=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );
_MC_SetPosition_J3(
	Axis:= J3, 
	Execute:= _MC_SetPosition_J3.Execute AND NOT _MC_SetPosition_J3.Done AND NOT _MC_SetPosition_J3.Error, 
	Position:= 0.0, 
	Mode:= FALSE, // true=relative false=absolute
	Done=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );
_MC_SetPosition_J4(
	Axis:= J4, 
	Execute:= _MC_SetPosition_J4.Execute AND NOT _MC_SetPosition_J4.Done AND NOT _MC_SetPosition_J4.Error, 
	Position:= 0.0, 
	Mode:= FALSE, // true=relative false=absolute
	Done=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );
_MC_SetPosition_J5(
	Axis:= J5, 
	Execute:= _MC_SetPosition_J5.Execute AND NOT _MC_SetPosition_J5.Done AND NOT _MC_SetPosition_J5.Error, 
	Position:= 0.0, 
	Mode:= FALSE, // true=relative false=absolute
	Done=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );
_MC_SetPosition_J6(
	Axis:= J6, 
	Execute:= _MC_SetPosition_J6.Execute AND NOT _MC_SetPosition_J6.Done AND NOT _MC_SetPosition_J6.Error, 
	Position:= 0.0, 
	Mode:= FALSE, // true=relative false=absolute
	Done=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );
END_FUNCTION_BLOCK

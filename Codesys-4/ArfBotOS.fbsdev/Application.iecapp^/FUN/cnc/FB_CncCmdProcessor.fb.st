FUNCTION_BLOCK FB_CncCmdProcessor EXTENDS FB_Module
VAR_INPUT
	GapVelocity: LREAL := 50.0;
	AvoidGaps: BOOL := FALSE;
	GapAccel: LREAL := 100.0;
    GapJerk: LREAL := 1000.0;
	Command: STRING(GVL_App.COMMAND_LEN):='';
END_VAR
VAR_IN_OUT
	J1, J2, J3, J4, J5, J6: AXIS_REF_SM3;
	AxisGroup: AXIS_GROUP_REF_SM3;
	CmdIO: ST_LogicCmdIO;
END_VAR
VAR_OUTPUT
    GCodeLines: ARRAY[0..SMC_GCodeViewer.c_uiLines] OF STRING;
END_VAR
VAR
    _OriConvention: SMC_ORI_CONVENTION := SMC_ORI_CONVENTION.ZYX; // _SMC_Trafo_ArticulatedRobot_6DOF is always ZYX (yaw pitch roll) according to https://content.helpme-codesys.com/en/CODESYS%20SoftMotion/_sm_trafo_pou_six_dof_robot.html
	_SMC_TrafoConfig_ArticulatedRobot_6DOF: SMC_TrafoConfig_ArticulatedRobot_6DOF;
	_SMC_Trafo_ArticulatedRobot_6DOF: SMC_Trafo_ArticulatedRobot_6DOF;
	_SMC_ControlAxisByPos_J1: SMC_ControlAxisByPos;
	_SMC_ControlAxisByPos_J2: SMC_ControlAxisByPos;
	_SMC_ControlAxisByPos_J3: SMC_ControlAxisByPos;
	_SMC_ControlAxisByPos_J4: SMC_ControlAxisByPos;
	_SMC_ControlAxisByPos_J5: SMC_ControlAxisByPos;
	_SMC_ControlAxisByPos_J6: SMC_ControlAxisByPos;
	_SMC_Interpolator: SMC_Interpolator;
    _SMC_GCodeViewer: SMC_GCodeViewer;
    _SMC_GCODEVIEWER_DATA:  ARRAY[0..110] OF SMC_GCODEVIEWER_DATA; // must be at least interpreter queue + preprocessors
	_TrackPosition: FB_PositionTracker;
	_PlanPath: FB_CncPathPlanner;
	//_PlanPath: REFERENCE TO CncPathPlanner;
	_SysLTime, _SysLTimeLast, _CycleTime: LTIME;
    _dwCycleTime: DWORD;
	_ErrorRtrig: R_TRIG;
    _i, _j, _k: INT;
END_VAR
VAR CONSTANT
	wVarCount : WORD := 1;
    MCODE_INPUT_WAIT: INT:=200;
    MCODE_INPUT_WAIT_START: INT:=MCODE_INPUT_WAIT+GVL_App.DIG_IN_ARR_BEG;
    MCODE_INPUT_WAIT_END: INT:=MCODE_INPUT_WAIT+GVL_App.DIG_IN_ARR_END;
    MCODE_OUTPUT_SET: INT:=300;
    MCODE_OUTPUT_SET_START: INT:=MCODE_OUTPUT_SET+GVL_App.DIG_OUT_ARR_BEG;
    MCODE_OUTPUT_SET_END: INT:=MCODE_OUTPUT_SET+GVL_App.DIG_OUT_ARR_END;
    MCODE_OUTPUT_CLR: INT:=400;
    MCODE_OUTPUT_CLR_START: INT:=MCODE_OUTPUT_CLR+GVL_App.DIG_OUT_ARR_BEG;
    MCODE_OUTPUT_CLR_END: INT:=MCODE_OUTPUT_CLR+GVL_App.DIG_OUT_ARR_END;
    MCODE_FLAG_WAIT: INT:=500;
    MCODE_FLAG_WAIT_START: INT:=MCODE_FLAG_WAIT+GVL_App.FLG_ARR_BEG;
    MCODE_FLAG_WAIT_END: INT:=MCODE_FLAG_WAIT+GVL_App.FLG_ARR_END;
END_VAR
VAR CONSTANT
	_STEP_DONE: 		INT := 1000;
	_STEP_IDLE: 		INT := 0000;
	_STEP_COMMAND: 		INT := 0010;
	_STEP_ONE: 			INT := 0020;
    _STEP_ERROR:        INT := 9999;
END_VAR
VAR
	_UnitModeCurrentState: PACK_ML.State;
	_CncCmdParser: FB_ParseCncCmd;
END_VAR
VAR
	lrR1 : LREAL:=30;
	
	astVar : ARRAY [0..wVarCount] OF SMC_SingleVar :=
			[(strVarName:='R1', pAdr := ADR(lrR1), eVarType := SMC_VarType.SMC_TYPE_LREAL)];
			
	stVarList : SMC_VarList := (wNumberVars := wVarCount, psvVarlist := ADR(astVar));
END_VAR

// NOTE currently the settings for J6 invert direction under scaling/mapping contradict using motion control 
// libraries and CNC. For CNC to work properly, the setting is required to be enabled (inverted). Additionally
// the homing direction and position will need to be negated. This all requires a full download. Will be leaving 
// setting disabled for now so that we can continue to use motion control. 
//
// to recap, take these steps if you would like to try out CNC:
// 1. open the axis pool settings and under the scaling/mapping section enable 'invert direction'
// 2. in the A_Homing POU, change the homing direction to positive.
// 3. in _00_MAIN POU, under FirstScan set the initial home offset to a positive value (keep the value and just change the sign).
// 4. do a full download.
// 5. go online and set the persistent value (if previously initialized in FirstScan) to a positive value.
//
// UPDATE: Figured out a work around for this issue. In the A_Interpolater POU, I negated the set position in the ControlAxisByPos
// function block for J6 such that it drives the opposite way. This appears to have resolved the issue. Why this is needed, I do not
// know. I am sure it is some minor overlooked detail somewhere in the code...

// Fusion 360 Post Processor Guide: https://cam.autodesk.com/posts/posts/guides/Post%20Processor%20Training%20Guide.pdf

// path planner has to be called in a separate task with a lower priority
// https://content.helpme-codesys.com/en/libs/SM3_CNC/Current/SM_CNC_POUs/NC2/Interpreter/SMC_NCInterpreter.html
//_PlanPath REF= _60_PathPlanner.PlanPath;

SUPER^();

// PackML state machine implementation
_UnitModeCurrentState := SUPER^._UnitMode.CurrentState;

//
CASE SUPER^._SeqMain OF
    _STEP_DONE:
		_SMC_Interpolator.bExecute := FALSE;
		SUPER^._SeqMain := _STEP_IDLE;
        
    _STEP_IDLE:
		// hang out here until called to start
	
	_STEP_COMMAND:
		//
		_CncCmdParser(Command:=Command);
		_PlanPath.M_SetOriConvention(OriConvention:=_OriConvention);//_CncCmdParser.stCommand.OriConvention);
		_PlanPath.M_SetStartPosition(StartPos:=_TrackPosition.Position);//_CncCmdParser.stCommand.StartPos);
		_PlanPath.M_Execute(Filename:=_CncCmdParser.stCommand.Filename); // TODO I think we should be careful here. May need to sync CheckVelocities with the Interpolator
		
		//
		IF NOT _CncCmdParser.Error THEN
			_SMC_Interpolator.bExecute := TRUE;
			SUPER^._SeqMain := _STEP_ONE;
		ELSE
			SUPER^.M_Error(Description:=CONCAT('Command parsing error: ',_CncCmdParser.ErrorDescription));
			SUPER^._SeqMain := _STEP_ERROR;
		END_IF
        
    _STEP_ONE:
		IF NOT _SMC_Interpolator.bBusy AND NOT SUPER^.Error THEN
			// next, please
			SUPER^._SeqMain := _STEP_DONE;
		END_IF
        
    _STEP_ERROR:
        // hang out here so we dont spam errors

END_CASE

// TODO these should reference INOUT AxisGroup but somehow wont compile like that. 
// using global scoped vars for now
_SMC_TrafoConfig_ArticulatedRobot_6DOF.a1:=ArfBotAxisGroup.Trafo.a1;//AxisGroup.Trafo.a1;
_SMC_TrafoConfig_ArticulatedRobot_6DOF.a2:=ArfBotAxisGroup.Trafo.a2;//AxisGroup.Trafo.a2;
_SMC_TrafoConfig_ArticulatedRobot_6DOF.a3:=ArfBotAxisGroup.Trafo.a3;//AxisGroup.Trafo.a3;
// conflicting documentation on if d1 needs to be pos or neg. only works as negative otherwise trafo reports invalid configuration
// https://product-help.schneider-electric.com/Machine%20Expert/V2.0/zh/codesys_softmotion/codesys_softmotion/modules/_sm_trafo_pou_six_dof_robot.html
// https://forge.codesys.com/forge/talk/Motion/thread/16e1f0327f/
_SMC_TrafoConfig_ArticulatedRobot_6DOF.d1:=-ArfBotAxisGroup.Trafo.d1;//-AxisGroup.Trafo.d1;
_SMC_TrafoConfig_ArticulatedRobot_6DOF.d3:=ArfBotAxisGroup.Trafo.d3;//AxisGroup.Trafo.d3;
_SMC_TrafoConfig_ArticulatedRobot_6DOF.d4:=ArfBotAxisGroup.Trafo.d4;//AxisGroup.Trafo.d4;
_SMC_TrafoConfig_ArticulatedRobot_6DOF.d6:=ArfBotAxisGroup.Trafo.d6;//AxisGroup.Trafo.d6;
_SMC_TrafoConfig_ArticulatedRobot_6DOF.q3_max_deg:=160; // J4 // TODO put magic numbers somewhere appropriate
_SMC_TrafoConfig_ArticulatedRobot_6DOF.q3_min_deg:=-160; // J4 // TODO put magic numbers somewhere appropriate
_SMC_TrafoConfig_ArticulatedRobot_6DOF.q5_max_deg:=160; // J6 // TODO put magic numbers somewhere appropriate
_SMC_TrafoConfig_ArticulatedRobot_6DOF.q5_min_deg:=-160; // J6 // TODO put magic numbers somewhere appropriate

// TODO figure out how this var list can be used. for now its just hello world
// this gets commented out because it has to be called in a separate task with a lower priority.
_PlanPath(VarList:= stVarList, DataOut=> );

_SMC_Interpolator.bEmergency_Stop := _SMC_ControlAxisByPos_J1.bError OR _SMC_ControlAxisByPos_J1.bStopIpo
									OR _SMC_ControlAxisByPos_J2.bError OR _SMC_ControlAxisByPos_J2.bStopIpo
									OR _SMC_ControlAxisByPos_J3.bError OR _SMC_ControlAxisByPos_J3.bStopIpo
									OR _SMC_ControlAxisByPos_J4.bError OR _SMC_ControlAxisByPos_J4.bStopIpo
									OR _SMC_ControlAxisByPos_J5.bError OR _SMC_ControlAxisByPos_J5.bStopIpo
									OR _SMC_ControlAxisByPos_J6.bError OR _SMC_ControlAxisByPos_J6.bStopIpo;

// get the current measured cycle time in useconds
_SysLTime := LTIME();
_CycleTime := _SysLTime - _SysLTimeLast;
_dwCycleTime := LTIME_TO_DWORD(_CycleTime) / 1000;
_SysLTimeLast := _SysLTime;

_SMC_Interpolator(
	bExecute:= , 
	poqDataIn:= _PlanPath.DataOut, 
	bSlow_Stop:= , 
	bEmergency_Stop:= , 
	bWaitAtNextStop:= , 
	dOverride:= , 
	iVelMode:= , 
	dwIpoTime:= _dwCycleTime, //GVL_App.dwMainInterval,
	dLastWayPos:= , 
	bAbort:= NOT _SMC_Interpolator.bExecute, 
	bSingleStep:= , 
	bAcknM:= , 
	bQuick_Stop:= , 
	dQuickDeceleration:= , 
	dJerkMax:= , 
	dQuickStopJerk:= , 
	bSuppressSystemMFunctions:= , 
	bDone=> , 
	bBusy=> , 
	bError=> , 
	wErrorID=> , 
	piSetPosition=> , 
	iStatus=> , 
	bWorking=> , 
	iActObjectSourceNo=> , 
	dActObjectLength=> , 
	dActObjectLengthRemaining=> , 
	dVel=> , 
	vecActTangent=> , 
	iLastSwitch=> , 
	dwSwitches=> , 
	dWayPos=> , 
	wM=> , 
	adToolLength=> , 
	Act_Object=> );

// handle m codes
CASE WORD_TO_INT(_SMC_Interpolator.wM) OF
	0:	// reset acknowledgment
		_SMC_Interpolator.bAcknM := FALSE;
    
    // M200 - M215 wait on inputs 0 - 15    
	MCODE_INPUT_WAIT_START..MCODE_INPUT_WAIT_END:
        IF CmdIo.Input[WORD_TO_INT(_SMC_Interpolator.wM) - MCODE_INPUT_WAIT] THEN
            _SMC_Interpolator.bAcknM := TRUE;
        END_IF
    
    // M300 - M315 set outputs 0 - 15
	MCODE_OUTPUT_SET_START..MCODE_OUTPUT_SET_END:
        CmdIo.Output[WORD_TO_INT(_SMC_Interpolator.wM) - MCODE_OUTPUT_SET] := TRUE;
        _SMC_Interpolator.bAcknM := TRUE;
    
    // M400 - M415 clear outputs 0 - 15   
	MCODE_OUTPUT_CLR_START..MCODE_OUTPUT_CLR_END:
        CmdIo.Output[WORD_TO_INT(_SMC_Interpolator.wM) - MCODE_OUTPUT_CLR] := FALSE;
        _SMC_Interpolator.bAcknM := TRUE;
    
    // M500 - M515 wait on flags 0 - 49    
	MCODE_FLAG_WAIT_START..MCODE_FLAG_WAIT_END:
        IF SUPER^.Flags[WORD_TO_INT(_SMC_Interpolator.wM) - MCODE_FLAG_WAIT].Value THEN
            SUPER^.Flags[WORD_TO_INT(_SMC_Interpolator.wM) - MCODE_FLAG_WAIT].Value := FALSE;
            _SMC_Interpolator.bAcknM := TRUE;
        END_IF
        
END_CASE
 
_SMC_GCodeViewer(
	GCodeText:= _PlanPath.GcodeText, 
	bEnable:= _SMC_Interpolator.bBusy, 
	interpolatedObjectId:= _SMC_Interpolator.actObjectId, 
	uiNumberOfBufferEntries:=  SIZEOF(_SMC_GCODEVIEWER_DATA) / SIZEOF(SMC_GCODEVIEWER_DATA), 
	pBuffer:= ADR(_SMC_GCODEVIEWER_DATA), 
	bError=> , 
	bBusy=> , 
	wErrorID=> , 
	asGCode=> GCodeLines);

_TrackPosition(
    OriConvention:=_OriConvention,
	AxisGroup:=ArfBotAxisGroup,
	J1:=J1, J2:=J2, J3:=J3,
	J4:=J4, J5:=J5, J6:=J6,
	VisuStruct3DTrack=>,
	Arm=>,
	Elbow=>,
	Hand=>);
    
_SMC_Trafo_ArticulatedRobot_6DOF(
	config:= _SMC_TrafoConfig_ArticulatedRobot_6DOF, 
	pi:= _SMC_Interpolator.piSetPosition, 
	dQ0_old:= J1.fActPosition, 
	dQ1_old:= J2.fActPosition, 
	dQ2_old:= J3.fActPosition, 
	dQ3_old:= -J4.fActPosition,
	dQ4_old:= J5.fActPosition, 
	dQ5_old:= -J6.fActPosition, 
	armState:= _TrackPosition.Arm, 
	elbowState:= _TrackPosition.Elbow, 
	handState:= _TrackPosition.Hand, 
	dQ0=> , 
	dQ1=> , 
	dQ2=> , 
	dQ3=> , 
	dQ4=> , 
	dQ5=> , 
	bError=> , 
	result=> );
	
_SMC_ControlAxisByPos_J1(
	Axis:= J1, 
	iStatus:= _SMC_Interpolator.iStatus, 
	bEnable:= _SMC_Interpolator.bWorking, 
	bAvoidGaps:= AvoidGaps, 
	fSetPosition:= _SMC_Trafo_ArticulatedRobot_6DOF.dQ0, 
	fGapVelocity:= GapVelocity, 
	fGapAcceleration:= GapAccel, 
	fGapDeceleration:= GapAccel, 
	fGapJerk:= GapJerk, 
	bBusy=> , 
	bCommandAborted=> , 
	bError=> , 
	iErrorID=> , 
	bStopIpo=> );
	
_SMC_ControlAxisByPos_J2(
	Axis:= J2, 
	iStatus:= _SMC_Interpolator.iStatus, 
	bEnable:= _SMC_Interpolator.bWorking, 
	bAvoidGaps:= AvoidGaps, 
	fSetPosition:= _SMC_Trafo_ArticulatedRobot_6DOF.dQ1, 
	fGapVelocity:= GapVelocity, 
	fGapAcceleration:= GapAccel, 
	fGapDeceleration:= GapAccel, 
	fGapJerk:= GapJerk, 
	bBusy=> , 
	bCommandAborted=> , 
	bError=> , 
	iErrorID=> , 
	bStopIpo=> );
	
_SMC_ControlAxisByPos_J3(
	Axis:= J3, 
	iStatus:= _SMC_Interpolator.iStatus, 
	bEnable:= _SMC_Interpolator.bWorking, 
	bAvoidGaps:= AvoidGaps, 
	fSetPosition:= _SMC_Trafo_ArticulatedRobot_6DOF.dQ2, 
	fGapVelocity:= GapVelocity, 
	fGapAcceleration:= GapAccel, 
	fGapDeceleration:= GapAccel, 
	fGapJerk:= GapJerk, 
	bBusy=> , 
	bCommandAborted=> , 
	bError=> , 
	iErrorID=> , 
	bStopIpo=> );
	
_SMC_ControlAxisByPos_J4(
	Axis:= J4, 
	iStatus:= _SMC_Interpolator.iStatus, 
	bEnable:= _SMC_Interpolator.bWorking, 
	bAvoidGaps:= AvoidGaps, 
	fSetPosition:= -_SMC_Trafo_ArticulatedRobot_6DOF.dQ3,
	fGapVelocity:= GapVelocity, 
	fGapAcceleration:= GapAccel, 
	fGapDeceleration:= GapAccel, 
	fGapJerk:= GapJerk, 
	bBusy=> , 
	bCommandAborted=> , 
	bError=> , 
	iErrorID=> , 
	bStopIpo=> );
	
_SMC_ControlAxisByPos_J5(
	Axis:= J5, 
	iStatus:= _SMC_Interpolator.iStatus, 
	bEnable:= _SMC_Interpolator.bWorking, 
	bAvoidGaps:= AvoidGaps, 
	fSetPosition:= _SMC_Trafo_ArticulatedRobot_6DOF.dQ4, 
	fGapVelocity:= GapVelocity, 
	fGapAcceleration:= GapAccel, 
	fGapDeceleration:= GapAccel, 
	fGapJerk:= GapJerk, 
	bBusy=> , 
	bCommandAborted=> , 
	bError=> , 
	iErrorID=> , 
	bStopIpo=> );
	
_SMC_ControlAxisByPos_J6(
	Axis:= J6, 
	iStatus:= _SMC_Interpolator.iStatus, 
	bEnable:= _SMC_Interpolator.bWorking, 
	bAvoidGaps:= AvoidGaps, 
	fSetPosition:= -_SMC_Trafo_ArticulatedRobot_6DOF.dQ5,
	fGapVelocity:= GapVelocity, 
	fGapAcceleration:= GapAccel, 
	fGapDeceleration:= GapAccel, 
	fGapJerk:= GapJerk, 
	bBusy=> , 
	bCommandAborted=> , 
	bError=> , 
	iErrorID=> , 
	bStopIpo=> );
	
_ErrorRtrig();
_ErrorRtrig.CLK := _PlanPath.Error
					OR _SMC_Interpolator.bError
					OR _SMC_Trafo_ArticulatedRobot_6DOF.bError
					OR _SMC_ControlAxisByPos_J1.bError
					OR _SMC_ControlAxisByPos_J2.bError
					OR _SMC_ControlAxisByPos_J3.bError
					OR _SMC_ControlAxisByPos_J4.bError
					OR _SMC_ControlAxisByPos_J5.bError
					OR _SMC_ControlAxisByPos_J6.bError;
IF _ErrorRtrig.Q THEN
	IF _PlanPath.Error THEN
		SUPER^.M_Error(Description:=_PlanPath.ErrorDescription);
	END_IF
	IF _SMC_Interpolator.bError THEN
		SUPER^.M_Error(Description := CONCAT(' Interpolator Error: ',
                                CONCAT(TO_STRING(_SMC_Interpolator.wErrorID),
                                CONCAT(' ', SMC_ErrorString(_SMC_Interpolator.wErrorID,
                                            SMC_Language_Type.english)))));
	END_IF
	IF _SMC_Trafo_ArticulatedRobot_6DOF.bError THEN
		SUPER^.M_Error(Description := CONCAT(' Trafo Error: ', 'unknown'));
	END_IF
	IF _SMC_ControlAxisByPos_J1.bError THEN
        IF _SMC_ControlAxisByPos_J1.iErrorID = 183 THEN
            SUPER^.M_Error(Description := F_AxisErrorDetail(Axis := J1, AxisName := 'J1'));
        ELSE
            SUPER^.M_Error(Description :=   CONCAT(' ControlAxisByPos_J1 Error: ',
                                            CONCAT(TO_STRING(_SMC_ControlAxisByPos_J1.iErrorID),
                                            CONCAT(' ', SMC_ErrorString(_SMC_ControlAxisByPos_J1.iErrorID,
                                                    SMC_Language_Type.english)))));
        END_IF
	END_IF
	IF _SMC_ControlAxisByPos_J2.bError THEN
        IF _SMC_ControlAxisByPos_J2.iErrorID = 183 THEN
            SUPER^.M_Error(Description := F_AxisErrorDetail(Axis := J2, AxisName := 'J2'));
        ELSE
            SUPER^.M_Error(Description :=   CONCAT(' ControlAxisByPos_J2 Error: ',
                                            CONCAT(TO_STRING(_SMC_ControlAxisByPos_J2.iErrorID),
                                            CONCAT(' ', SMC_ErrorString(_SMC_ControlAxisByPos_J2.iErrorID,
                                                    SMC_Language_Type.english)))));
        END_IF
	END_IF
	IF _SMC_ControlAxisByPos_J3.bError THEN
        IF _SMC_ControlAxisByPos_J3.iErrorID = 183 THEN
            SUPER^.M_Error(Description := F_AxisErrorDetail(Axis := J3, AxisName := 'J3'));
        ELSE
            SUPER^.M_Error(Description :=   CONCAT(' ControlAxisByPos_J3 Error: ',
                                            CONCAT(TO_STRING(_SMC_ControlAxisByPos_J3.iErrorID),
                                            CONCAT(' ', SMC_ErrorString(_SMC_ControlAxisByPos_J3.iErrorID,
                                                    SMC_Language_Type.english)))));
        END_IF
	END_IF
	IF _SMC_ControlAxisByPos_J4.bError THEN
        IF _SMC_ControlAxisByPos_J4.iErrorID = 183 THEN
            SUPER^.M_Error(Description := F_AxisErrorDetail(Axis := J4, AxisName := 'J4'));
        ELSE
            SUPER^.M_Error(Description :=   CONCAT(' ControlAxisByPos_J4 Error: ',
                                            CONCAT(TO_STRING(_SMC_ControlAxisByPos_J4.iErrorID),
                                            CONCAT(' ', SMC_ErrorString(_SMC_ControlAxisByPos_J4.iErrorID,
                                                    SMC_Language_Type.english)))));
        END_IF
	END_IF
	IF _SMC_ControlAxisByPos_J5.bError THEN
        IF _SMC_ControlAxisByPos_J5.iErrorID = 183 THEN
            SUPER^.M_Error(Description := F_AxisErrorDetail(Axis := J5, AxisName := 'J5'));
        ELSE
            SUPER^.M_Error(Description :=   CONCAT(' ControlAxisByPos_J5 Error: ',
                                            CONCAT(TO_STRING(_SMC_ControlAxisByPos_J5.iErrorID),
                                            CONCAT(' ', SMC_ErrorString(_SMC_ControlAxisByPos_J5.iErrorID,
                                                    SMC_Language_Type.english)))));
        END_IF
	END_IF
	IF _SMC_ControlAxisByPos_J6.bError THEN
        IF _SMC_ControlAxisByPos_J6.iErrorID = 183 THEN
            SUPER^.M_Error(Description := F_AxisErrorDetail(Axis := J6, AxisName := 'J6'));
        ELSE
            SUPER^.M_Error(Description :=   CONCAT(' ControlAxisByPos_J6 Error: ',
                                            CONCAT(TO_STRING(_SMC_ControlAxisByPos_J6.iErrorID),
                                            CONCAT(' ', SMC_ErrorString(_SMC_ControlAxisByPos_J6.iErrorID,
                                                    SMC_Language_Type.english)))));
        END_IF
	END_IF
END_IF
END_FUNCTION_BLOCK

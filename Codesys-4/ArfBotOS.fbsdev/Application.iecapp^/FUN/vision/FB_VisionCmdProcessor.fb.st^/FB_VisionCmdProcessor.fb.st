FUNCTION_BLOCK FB_VisionCmdProcessor EXTENDS FB_Module
VAR_INPUT
	Command: STRING(GVL_App.COMMAND_LEN):='';
	LogicCmdIO: REFERENCE TO ST_LogicCmdIO;
	VisionSystemIpAddress: STRING(19):='127.0.0.1';
	VisionSystemPort: UINT:=50011;
END_VAR
VAR_OUTPUT
	VisionStateText: STRING:='READY';
END_VAR
VAR
	_CaptureImage_Output: STRING(255);
	_CaptureImage_WorkerUid: STRING;
	_LocatePart_Output: STRING(255);
	_LocatePart_WorkerUid: STRING;
	_RunCalibration_Output: STRING(255);
	_RunCalibration_WorkerUid: STRING;
    _Vision: FB_OpenCV;
    _VisionCommandParser: FB_ParseVisionCmd;
    _VisionDoneRtrig: R_TRIG;
	_VisionBusyRtrig: R_TRIG;
	_VisionResult: STRING(255);
	_ResultPosition: INT;
	_ResultPosition_CAL: INT;
	_ResultPosition_LOC: INT;
	_ResultPosition_CAP: INT;
	_SpacePosition: INT;
	_VisionObjectIndex: UINT;
	_VisionRatio: REAL;
	_VisionRotation: REAL;
	_VisionError: REAL;
	_PartsCount: UDINT;
	_UnitModeCurrentState: PACK_ML.State;
END_VAR

VAR CONSTANT
	_RESPONSE_CAL:          STRING:='res: CAL ';
	_RESPONSE_LOC:          STRING:='res: LOC ';
	_RESPONSE_CAP:          STRING:='res: CAP ';

	_STEP_DONE: 			INT := 1000;
	_STEP_IDLE: 			INT := 0000;
	_STEP_COMMAND: 			INT := 0020;
	_STEP_TRIGGER: 			INT := 0030;
	_STEP_WAIT: 			INT := 0040;
	_STEP_ERR: 				INT := 0050;
END_VAR

SUPER^();

// PackML state machine implementation
_UnitModeCurrentState := SUPER^._UnitMode.CurrentState;

//
CASE SUPER^._SeqMain OF
	
    _STEP_DONE:
		SUPER^._SeqMain := _STEP_IDLE;
        
    _STEP_IDLE:
		// hang out here until called to run
        
    _STEP_COMMAND: // parse vision command
        _VisionCommandParser(Command:=Command);
		IF _VisionCommandParser.Error THEN
			SUPER^.M_Error(Description:=_VisionCommandParser.ErrorDescription);
			//
			SUPER^._SeqMain := _STEP_ERR;
		ELSE
			IF _VisionCommandParser.stCommand.Timeout > 0 THEN
				SUPER^._TimeoutTimer.PT := UDINT_TO_TIME(_VisionCommandParser.stCommand.Timeout * 1000);
			END_IF
			//
			SUPER^._SeqMain := _STEP_TRIGGER;
		END_IF
		
	_STEP_TRIGGER: //
		CASE _VisionCommandParser.stCommand.CommandType OF
			DUT_Command_VisionType.Calibrate: //
				_Vision.M_CalibrateCamera(	iImageWidth:= _VisionCommandParser.stCommand.ImageWidth, 
											iImageHeight:= _VisionCommandParser.stCommand.ImageHeight, 
											fCalCheckerSize:= _VisionCommandParser.stCommand.Calibrate.CheckerSize, 
											iCalCheckerX:= _VisionCommandParser.stCommand.Calibrate.CheckerXCount, 
											iCalCheckerY:= _VisionCommandParser.stCommand.Calibrate.CheckerYCount,
                                            uTimeout:=_VisionCommandParser.stCommand.Timeout);
			
			DUT_Command_VisionType.Capture: //
				_Vision.M_CaptureImage(	iImageWidth:= _VisionCommandParser.stCommand.ImageWidth, 
										iImageHeight:= _VisionCommandParser.stCommand.ImageHeight,
                                        uTimeout:=_VisionCommandParser.stCommand.Timeout);
			
			DUT_Command_VisionType.LocatePart: //
                // clear the parts count register
				_PartsCount := 0;
				LogicCmdIO.Register[_VisionCommandParser.stCommand.Locate.ResultPosition] := 0;
                
                // call vision
				_Vision.M_LocatePart( 	iImageWidth:= _VisionCommandParser.stCommand.ImageWidth, 
										iImageHeight:= _VisionCommandParser.stCommand.ImageHeight, 
										iMaxPos:= _VisionCommandParser.stCommand.Locate.MaxParts, 
										dMaxOverlap:= _VisionCommandParser.stCommand.Locate.MaxOverlap, 
										dScore:= _VisionCommandParser.stCommand.Locate.Score, 
										dToleranceAngle:= _VisionCommandParser.stCommand.Locate.ToleranceAngle,
                                        uTimeout:=_VisionCommandParser.stCommand.Timeout,
										sTemplateName:=_VisionCommandParser.stCommand.Locate.TemplateName,
										sScriptFilename:=_VisionCommandParser.stCommand.Locate.ScriptFilename);
			
		END_CASE;
		
		//
		SUPER^._SeqMain := _STEP_WAIT;
		
	_STEP_WAIT: //
		IF _VisionDoneRtrig.Q THEN
			
			//
			_VisionResult := _Vision.P_Response;
			
			//
			_ResultPosition_CAL := FIND(STR1:=_VisionResult,STR2:=_RESPONSE_CAL);
			_ResultPosition_LOC := FIND(STR1:=_VisionResult,STR2:=_RESPONSE_LOC);
			_ResultPosition_CAP := FIND(STR1:=_VisionResult,STR2:=_RESPONSE_CAP);
			
			//
			IF _ResultPosition_CAL > 0 THEN 
				// calibration was completed
				// res: CAL rat:23.23 rot:23.23 err:23.23
				_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-LEN(_RESPONSE_CAL),POS:=LEN(_RESPONSE_CAL)+1);
				//VisionStateText := CONCAT(VisionStateText, '\n');
				//VisionStateText := CONCAT(VisionStateText, _VisionResult);
				// rat:23.23 rot:23.23 err:23.23
				_ResultPosition := FIND(STR1:=_VisionResult,STR2:='rat:');
				IF _ResultPosition > 0 THEN
					_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-LEN('rat:'),POS:=LEN('rat:')+1);
					_SpacePosition := FIND(STR1:=_VisionResult,STR2:=' ');
					_VisionRatio := STRING_TO_REAL(MID(STR:=_VisionResult,LEN:=_SpacePosition-1,POS:=1));
					_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-_SpacePosition,POS:=_SpacePosition+1);
				END_IF
				// rot:23.23 err:23.23
				_ResultPosition := FIND(STR1:=_VisionResult,STR2:='rot:');
				IF _ResultPosition > 0 THEN
					_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-LEN('rot:'),POS:=LEN('rot:')+1);
					_SpacePosition := FIND(STR1:=_VisionResult,STR2:=' ');
					_VisionRotation := STRING_TO_REAL(MID(STR:=_VisionResult,LEN:=_SpacePosition-1,POS:=1));
					_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-_SpacePosition,POS:=_SpacePosition+1);
				END_IF
				// err:23.23
				_ResultPosition := FIND(STR1:=_VisionResult,STR2:='err:');
				IF _ResultPosition > 0 THEN
					_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-LEN('err:'),POS:=LEN('err:')+1);
					_SpacePosition := FIND(STR1:=_VisionResult,STR2:=' ');
					_VisionError := STRING_TO_REAL(MID(STR:=_VisionResult,LEN:=_SpacePosition-1,POS:=1));
					_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-_SpacePosition,POS:=_SpacePosition+1);
				END_IF
				
			ELSIF _ResultPosition_LOC > 0 THEN 
				// part was located
				// 'res: LOC obj:0 cx:54.66 cy:119.45 a:-12.87 s:0.84 $NLOC obj:1 cx:71.49 cy:100.06 a:10.13 s:0.83 $N'
				_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-LEN(_RESPONSE_LOC),POS:=LEN(_RESPONSE_LOC)+1);
				//VisionStateText := CONCAT(VisionStateText, '\n');
				//VisionStateText := CONCAT(VisionStateText, _VisionResult);
				_VisionObjectIndex := 0;
				WHILE FIND(STR1:=_VisionResult,STR2:='obj:') > 0 
					AND _VisionObjectIndex <= _VisionCommandParser.stCommand.Locate.MaxParts 
					AND _VisionObjectIndex + _VisionCommandParser.stCommand.Locate.ResultPosition < GVL_App.VIS_ARR_END DO
					// 'obj:0 cx:54.66 cy:119.45 a:-12.87 s:0.84 $NLOC obj:1 cx:71.49 cy:100.06 a:10.13 s:0.83 $N'
					// $NLOC obj:1 cx:71.49 cy:100.06 a:10.13 s:0.83 $N'
					_ResultPosition := FIND(STR1:=_VisionResult,STR2:='obj:');
					IF _ResultPosition > 0 THEN
						_PartsCount := _PartsCount + 1;
						_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-_ResultPosition-LEN('obj:')+1,POS:=_ResultPosition+LEN('obj:'));
						_SpacePosition := FIND(STR1:=_VisionResult,STR2:=' ');
						_VisionObjectIndex := STRING_TO_UINT(MID(STR:=_VisionResult,LEN:=_SpacePosition-1,POS:=1));
						SUPER^.VisionRegisters[_VisionCommandParser.stCommand.Locate.ResultPosition + _VisionObjectIndex].Comment := CONCAT('obj:', UDINT_TO_STRING(_VisionObjectIndex));
						_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-_SpacePosition,POS:=_SpacePosition+1);
					END_IF
					// cx:54.66 cy:119.45 a:-12.87 s:0.84 $NLOC obj:1 cx:71.49 cy:100.06 a:10.13 s:0.83 $N'
					_ResultPosition := FIND(STR1:=_VisionResult,STR2:='cx:');
					IF _ResultPosition > 0 THEN
						_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-LEN('cx:'),POS:=LEN('cx:')+1);
						_SpacePosition := FIND(STR1:=_VisionResult,STR2:=' ');
						SUPER^.VisionRegisters[_VisionCommandParser.stCommand.Locate.ResultPosition + _VisionObjectIndex].Position.X := STRING_TO_LREAL(MID(STR:=_VisionResult,LEN:=_SpacePosition-1,POS:=1));
						_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-_SpacePosition,POS:=_SpacePosition+1);
					END_IF
					// cy:119.45 a:-12.87 s:0.84 $NLOC obj:1 cx:71.49 cy:100.06 a:10.13 s:0.83 $N'
					_ResultPosition := FIND(STR1:=_VisionResult,STR2:='cy:');
					IF _ResultPosition > 0 THEN
						_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-LEN('cy:'),POS:=LEN('cy:')+1);
						_SpacePosition := FIND(STR1:=_VisionResult,STR2:=' ');
						SUPER^.VisionRegisters[_VisionCommandParser.stCommand.Locate.ResultPosition + _VisionObjectIndex].Position.Y := STRING_TO_LREAL(MID(STR:=_VisionResult,LEN:=_SpacePosition-1,POS:=1));
						_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-_SpacePosition,POS:=_SpacePosition+1);
					END_IF
					// a:-12.87 s:0.84 $NLOC obj:1 cx:71.49 cy:100.06 a:10.13 s:0.83 $N'
					_ResultPosition := FIND(STR1:=_VisionResult,STR2:='a:');
					IF _ResultPosition > 0 THEN
						_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-LEN('a:'),POS:=LEN('a:')+1);
						_SpacePosition := FIND(STR1:=_VisionResult,STR2:=' ');
						SUPER^.VisionRegisters[_VisionCommandParser.stCommand.Locate.ResultPosition + _VisionObjectIndex].Position.A := STRING_TO_LREAL(MID(STR:=_VisionResult,LEN:=_SpacePosition-1,POS:=1));
						_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-_SpacePosition,POS:=_SpacePosition+1);
					END_IF
					// s:0.84 $NLOC obj:1 cx:71.49 cy:100.06 a:10.13 s:0.83 $N'
					_ResultPosition := FIND(STR1:=_VisionResult,STR2:='s:');
					IF _ResultPosition > 0 THEN
						_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-LEN('s:'),POS:=LEN('s:')+1);
						_SpacePosition := FIND(STR1:=_VisionResult,STR2:=' ');
						SUPER^.VisionRegisters[_VisionCommandParser.stCommand.Locate.ResultPosition + _VisionObjectIndex].Scaling := STRING_TO_REAL(MID(STR:=_VisionResult,LEN:=_SpacePosition-1,POS:=1));
						_VisionResult := MID(STR:=_VisionResult,LEN:=LEN(_VisionResult)-_SpacePosition,POS:=_SpacePosition+1);
					END_IF
					// $NLOC obj:1 cx:71.49 cy:100.06 a:10.13 s:0.83 $N'
				END_WHILE
				// return how many part were found
				LogicCmdIO.Register[_VisionCommandParser.stCommand.Locate.ResultPosition] := _PartsCount;
				
			ELSIF _ResultPosition_CAP > 0 THEN 
				// image was captured
				
			END_IF
			
			//
			SUPER^._SeqMain := _STEP_DONE;
			
		END_IF

END_CASE

//
_Vision(sIpAddress:= VisionSystemIpAddress, uPort:= VisionSystemPort);

//
_VisionBusyRtrig(clk:=_Vision.P_Busy);
IF _VisionBusyRtrig.Q THEN
	VisionStateText := 'Busy';
END_IF

//
_VisionDoneRtrig(clk:=_Vision.P_Done);
IF _VisionDoneRtrig.Q THEN
	VisionStateText := 'Done';
    
    IF _ResultPosition_CAL > 0 THEN
        VisionStateText := CONCAT(VisionStateText, ': Calibration Complete');
        //VisionStateText := CONCAT(VisionStateText, _Vision.P_Response); // TODO add the response output?
        
    ELSIF _ResultPosition_LOC > 0 THEN 
        VisionStateText := CONCAT(VisionStateText, ': Found ');
        VisionStateText := CONCAT(VisionStateText, UDINT_TO_STRING(_PartsCount));
        VisionStateText := CONCAT(VisionStateText, ' parts');
        
    ELSIF _ResultPosition_CAP > 0 THEN
        VisionStateText := CONCAT(VisionStateText, ': Capture complete');
        
    END_IF
END_IF
END_FUNCTION_BLOCK

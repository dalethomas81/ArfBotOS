FUNCTION_BLOCK FB_ParseVisionCmd
VAR_INPUT
    Command: STRING(GVL_App.COMMAND_LEN);
END_VAR
VAR_OUTPUT
    stCommand: ST_VisionCmd;
	Error: BOOL;
	ErrorDescription: STRING;
END_VAR
VAR
	_CmdParams: ARRAY [0..GVL_App.COMMAND_LEN] OF STRING;
    _CommandType: STRING;
	_StrPosition: INT;
END_VAR
VAR CONSTANT
	_CommandTypeCal: STRING(3):='CAL'; // calibrate vision
	_CommandTypeCap: STRING(3):='CAP'; // capture image
	_CommandTypeAcq: STRING(3):='ACQ'; // legacy alias for CAP
	_CommandTypeLoc: STRING(3):='LOC'; // locate part
END_VAR

// CAL|CAP|LOC ImageWidth ImageHeight Timeout

// CAL Calibrate vision system
    // CheckerSize (user units) XCount YCount 
    // CAL 2016 2000 60 25.0 6 9
    
// CAP Capture image
    // CAP 2016 2000 60
    // ACQ is still accepted as a legacy alias
    
// LOC Locate part 
    // MaxParts MaxOverlap Score ToleranceAngle ResultPostion TemplateName ScriptName
    // LOC 2016 2000 60 10 0.0 0.8 90.0 1 template.jpg FastTemplateMatching.py
	// MaxParts - max number of parts to find in image
	// MaxOverlap - max overlap of parts in image
	// Score - min required score for parts
	// ToleranceAngle - +/- search angle (the larger the number, the longer the search)
	// ResultPostion - index of vision register to store result
	// TemplateName - name of the template to search for
	// ScriptName - name of the template to search for

Error:=FALSE;
ErrorDescription:='';

// split command string into commands
_CmdParams:=F_SplitCmd(Command:=Command);

_CommandType:= _CmdParams[0];
IF _CommandType = _CommandTypeCal THEN
    // CAL 2016 2000 0060 025.0 0006 0009
	stCommand.CommandType:=DUT_Command_VisionType.Calibrate;
    stCommand.Calibrate.CheckerSize:=STRING_TO_REAL(_CmdParams[4]);
    stCommand.Calibrate.CheckerXCount:=STRING_TO_UINT(_CmdParams[5]);
    stCommand.Calibrate.CheckerYCount:=STRING_TO_UINT(_CmdParams[6]);
	
	IF stCommand.Calibrate.CheckerSize <= 0
		OR stCommand.Calibrate.CheckerXCount <= 0
		OR stCommand.Calibrate.CheckerYCount <= 0 THEN
			Error:=TRUE;
			ErrorDescription:='Checkerboard configuration incorrect';
			RETURN;
	END_IF
    
ELSIF (_CommandType = _CommandTypeCap) OR (_CommandType = _CommandTypeAcq) THEN
    // CAP 2016 2000 0060
	stCommand.CommandType:=DUT_Command_VisionType.Capture;
    
ELSIF _CommandType = _CommandTypeLoc THEN
    // LOC 2016 2000 0060 0010 000.0 0.8 090.0 001 template.jpg FastTemplatMatching.py
	stCommand.CommandType:=DUT_Command_VisionType.LocatePart;
    stCommand.Locate.MaxParts:=STRING_TO_UINT(_CmdParams[4]);
    stCommand.Locate.MaxOverlap:=STRING_TO_REAL(_CmdParams[5]);
    stCommand.Locate.Score:=STRING_TO_REAL(_CmdParams[6]);
    stCommand.Locate.ToleranceAngle:=STRING_TO_REAL(_CmdParams[7]);
    stCommand.Locate.ResultPosition:=STRING_TO_UINT(_CmdParams[8]);
    stCommand.Locate.TemplateName:=_CmdParams[9];
    stCommand.Locate.ScriptFilename:=_CmdParams[10];
	
	IF stCommand.Locate.MaxParts <= 0
		OR stCommand.Locate.ResultPosition < GVL_App.VIS_ARR_BEG
		OR stCommand.Locate.ResultPosition > GVL_App.VIS_ARR_END
		OR stCommand.Locate.TemplateName = '' THEN
			Error:=TRUE;
			ErrorDescription:='Locate part configuration incorrect';
			RETURN;
	END_IF
	
ELSE
	Error:=TRUE;
	ErrorDescription:='Unknown command type for vision';
	RETURN;
    
END_IF

// common
stCommand.ImageWidth:=STRING_TO_UINT(_CmdParams[1]);
stCommand.ImageHeight:=STRING_TO_UINT(_CmdParams[2]);
stCommand.Timeout:=STRING_TO_UDINT(_CmdParams[3]);

IF stCommand.ImageWidth <= 0
	OR stCommand.ImageHeight <= 0 THEN
		Error:=TRUE;
		ErrorDescription:='Image size settings incorrect';
END_IF
END_FUNCTION_BLOCK

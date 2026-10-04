FUNCTION F_BuildVisionCmd : STRING(GVL_App.COMMAND_LEN)
VAR_INPUT
    stCommand: ST_VisionCmd;
END_VAR
VAR
    _Command: STRING(GVL_App.COMMAND_LEN);
END_VAR
VAR CONSTANT
	_Delimiter: STRING:=' ';
	_CommandTypeCal: STRING(3):='CAL'; // calibrate vision
	_CommandTypeCap: STRING(3):='CAP'; // capture image
	_CommandTypeLoc: STRING(3):='LOC'; // locate part
END_VAR

// CAL|CAP|LOC ImageWidth ImageHeight Timeout

// CAL Calibrate vision system
    // CheckerSize (user units) XCount YCount 
    // CAL 2016 2000 60 25.0 6 9
    
// CAP Capture image
    // CAP 2016 2000 60
    
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

// clear the command
_Command:='';

// build the command
CASE stCommand.CommandType OF
	DUT_Command_VisionType.Calibrate:
		F_AddElement(_Command, _Delimiter, _CommandTypeCal);
	DUT_Command_VisionType.Capture:
		F_AddElement(_Command, _Delimiter, _CommandTypeCap);
	DUT_Command_VisionType.LocatePart:
		F_AddElement(_Command, _Delimiter, _CommandTypeLoc);
END_CASE

// add the common items width, height, timeout
F_AddElement(_Command, _Delimiter, UINT_TO_STRING(stCommand.ImageWidth));
F_AddElement(_Command, _Delimiter, UINT_TO_STRING(stCommand.ImageHeight));
F_AddElement(_Command, _Delimiter, UDINT_TO_STRING(stCommand.Timeout));

//
CASE stCommand.CommandType OF
	DUT_Command_VisionType.Calibrate:
		// CAL 2016 2000 60 25.0 6 9
		F_AddElement(_Command, _Delimiter, REAL_TO_STRING(stCommand.Calibrate.CheckerSize));
		F_AddElement(_Command, _Delimiter, UINT_TO_STRING(stCommand.Calibrate.CheckerXCount));
		F_AddElement(_Command, _Delimiter, UINT_TO_STRING(stCommand.Calibrate.CheckerYCount));
		
	DUT_Command_VisionType.Capture:
		// CAP 2016 2000 60
		// we are done
		
	DUT_Command_VisionType.LocatePart:
		// LOC 2016 2000 60 10 0.0 0.8 90.0 1 template.jpg FastTemplateMatching.py
		F_AddElement(_Command, _Delimiter, UINT_TO_STRING(stCommand.Locate.MaxParts));
		F_AddElement(_Command, _Delimiter, REAL_TO_STRING(stCommand.Locate.MaxOverlap));
		F_AddElement(_Command, _Delimiter, REAL_TO_STRING(stCommand.Locate.Score));
		F_AddElement(_Command, _Delimiter, REAL_TO_STRING(stCommand.Locate.ToleranceAngle));
		F_AddElement(_Command, _Delimiter, UINT_TO_STRING(stCommand.Locate.ResultPosition));
		F_AddElement(_Command, _Delimiter, stCommand.Locate.TemplateName);
		F_AddElement(_Command, _Delimiter, stCommand.Locate.ScriptFilename);
		
END_CASE

// set function equal to command
F_BuildVisionCmd := _Command;
END_FUNCTION

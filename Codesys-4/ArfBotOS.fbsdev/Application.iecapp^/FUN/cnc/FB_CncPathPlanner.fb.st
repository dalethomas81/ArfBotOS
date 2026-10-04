FUNCTION_BLOCK FB_CncPathPlanner
VAR_IN_OUT
	VarList: SMC_VarList;
END_VAR
VAR_INPUT
END_VAR
VAR_OUTPUT
	DataOut: POINTER TO SMC_OUTQUEUE;
	Busy: BOOL;
	Done: BOOL;
	Error: BOOL;
	ErrorDescription: STRING;
    GcodeText: SMC_GCODE_TEXT;
END_VAR
VAR
	_Filename: STRING := '_cnc/CNC_FromFile.cnc';
	_SMC_ReadNCFile2: SMC_ReadNCFile2;
	_SMC_NCInterpreter: SMC_NCInterpreter;
    _SMC_SmoothPath: SMC_SmoothPath;
	_SMC_CheckVelocities: SMC_CheckVelocities;
	_SMC_GEOINFO_Ipo: ARRAY[0..50] OF SMC_GEOINFO;
	_SMC_GEOINFO_Smooth: ARRAY[0..50] OF SMC_GEOINFO;
	_StartPos: SMC_PosInfo;
	_OriConvention: SMC_ORI_CONVENTION := SMC_ORI_CONVENTION.ZYZ;
	_ExecuteRtrig: R_TRIG;
	_Execute: BOOL;
	_ResetRtrig: R_TRIG;
	_Reset: BOOL;
	_Sequence: INT:=0;
	_ErrorRtrig: R_TRIG;
END_VAR
VAR CONSTANT
	_BaseCncDirectory: STRING:='_cnc/'; // before update to 3.5.19.40 '/var/opt/codesys/_cnc/'
END_VAR

_ExecuteRtrig(CLK:=_Execute);
IF _ExecuteRtrig.Q THEN
	_Execute:= FALSE;
END_IF
_ResetRtrig(CLK:=_Reset);
IF _ResetRtrig.Q THEN
	_Reset:= FALSE;
	_Sequence:=999;
END_IF

Busy := _Sequence <> 000;

CASE _Sequence OF
	999:
		_SMC_ReadNCFile2.bExecute := FALSE;
		_Sequence := 000;
	000:
		IF _ExecuteRtrig.Q THEN
			_SMC_ReadNCFile2.bExecute := TRUE;
			_Sequence := 1;
		END_IF
	001:
		IF NOT _SMC_CheckVelocities.bBusy THEN
			_Sequence := 999;
			Done := TRUE;
		END_IF
	
END_CASE

_SMC_ReadNCFile2(
	bExecute:= , 
	bAbort:= _Sequence = 999, 
	sFileName:= _Filename, 
	pvl:= ADR(VarList), 
	fDefaultVel:= , 
	fDefaultAccel:= , 
	fDefaultDecel:= , 
	fDefaultVelFF:= , 
	fDefaultAccelFF:= , 
	fDefaultDecelFF:= , 
	b3DMode:= , 
	bStepSuppress:= , 
	aSubProgramDirs:= , 
	bParenthesesAsComments:= , 
	bDisableJumpBuffer:= , 
	pCustomFunTable:= , 
	aTokenModifier:= , 
	aSentenceModifier:= , 
	bBusy=> , 
	bError=> , 
	ErrorID=> , 
	errorPos=> , 
	ErrorProgramName=> , 
	sentences=> , 
	adwFileSize=> , 
	adwPos=> );

// https://content.helpme-codesys.com/en/libs/SM3_CNC/Current/SM_CNC_POUs/NC2/Interpreter/SMC_NCInterpreter.html
_SMC_NCInterpreter(
	sentences:= _SMC_ReadNCFile2.sentences, 
	bExecute:= _SMC_ReadNCFile2.bExecute, 
	bAbort:= _Sequence = 999, 
	bAppend:= , 
	piStartPosition:= _StartPos, 
	vStartToolLength:= , 
	nSizeOutQueue:= SIZEOF(_SMC_GEOINFO_Ipo), 
	pbyBufferOutQueue:= ADR(_SMC_GEOINFO_Ipo), 
	bEnableSyntaxChecks:= , 
	eOriConv:= _OriConvention,    
    // A/B/C as independent axes so G91 A-45 is signed and can pass ±180.
    // Tracker/trafo stay ZYX; do not set _OriConvention to ADDAXES (that feeds J4/J5/J6 as Euler and faults the group).
    //eOriConv:= SMC_ORI_CONVENTION.ADDAXES,
	dCircleTolerance:= , 
	pInterpreterStack:= , 
	nInterpreterStackSizeBytes:= , 
	bDone=> , 
	bBusy=> , 
	bError=> , 
	wErrorID=> , 
	errorPos=> , 
	poqDataOut=> , 
	iStatus=> , 
	iLineNumberDecoded=> , 
	GCodeText=> GcodeText, 
	CallstackInfo=> , 
	aActivePrograms=> );
    
_SMC_SmoothPath(
	bExecute:= _SMC_ReadNCFile2.bExecute, 
	bAbort:= _Sequence = 999, 
	bAppend:= , 
	poqDataIn:= _SMC_NCInterpreter.poqDataOut, 
	dEdgeDistance:= , 
	dAngleTol:= , 
	nSizeOutQueue:= SIZEOF(_SMC_GEOINFO_Smooth), 
	pbyBufferOutQueue:= ADR(_SMC_GEOINFO_Smooth), 
	eMode:= , 
	bSymmetricalDistances:= , 
	bImprovedSymmetricCuts:= , 
	eAddAxMode:= , 
	dMinimumCurvatureRadius:= , 
	bCheckCurvature:= , 
	dRelativeCurvatureTol:= , 
	bCheckAddAxVelJump:= , 
	dMaxAddAxVelDifference:= , 
	bDone=> , 
	bBusy=> , 
	bError=> , 
	wErrorID=> , 
	poqDataOut=> , 
	udiStopsDueToCurvatureRadius=> );

_SMC_CheckVelocities(
	bExecute:= _SMC_NCInterpreter.bExecute, 
	bAbort:= _Sequence = 999, 
	poqDataIn:= _SMC_SmoothPath.poqDataOut, 
	dAngleTol:= , 
	bCheckAddAxVelJump:= , 
	dMaxAddAxVelDifference:= , 
	bBusy=> , 
	bError=> , 
	wErrorID=> , 
	poqDataOut=> DataOut);
	
_ErrorRtrig();
_ErrorRtrig.CLK:= _SMC_ReadNCFile2.bError
				OR _SMC_NCInterpreter.bError
				OR _SMC_SmoothPath.bError
				OR _SMC_CheckVelocities.bError;
IF _ErrorRtrig.Q THEN
	Error := TRUE;
	IF _SMC_ReadNCFile2.bError THEN
		ErrorDescription := CONCAT(' ReadNCFile Error: ',
                                CONCAT(TO_STRING(_SMC_ReadNCFile2.ErrorID),
                                CONCAT(' ', SMC_ErrorString(_SMC_ReadNCFile2.ErrorID,
                                            SMC_Language_Type.english)))) ;
	END_IF
	IF _SMC_NCInterpreter.bError THEN
		ErrorDescription := CONCAT(' NCInterpreter Error: ',
                                CONCAT(TO_STRING(_SMC_NCInterpreter.wErrorID),
                                CONCAT(' ', SMC_ErrorString(_SMC_NCInterpreter.wErrorID,
                                            SMC_Language_Type.english)))) ;
	END_IF
	IF _SMC_SmoothPath.bError THEN
		ErrorDescription := CONCAT(' SmoothPath Error: ',
                                CONCAT(TO_STRING(_SMC_SmoothPath.wErrorID),
                                CONCAT(' ', SMC_ErrorString(_SMC_SmoothPath.wErrorID,
                                            SMC_Language_Type.english)))) ;
	END_IF
	IF _SMC_CheckVelocities.bError THEN
		ErrorDescription := CONCAT(' CheckVelocities Error: ',
                                CONCAT(TO_STRING(_SMC_CheckVelocities.wErrorID),
                                CONCAT(' ', SMC_ErrorString(_SMC_CheckVelocities.wErrorID,
                                            SMC_Language_Type.english)))) ;
	END_IF
END_IF
END_FUNCTION_BLOCK

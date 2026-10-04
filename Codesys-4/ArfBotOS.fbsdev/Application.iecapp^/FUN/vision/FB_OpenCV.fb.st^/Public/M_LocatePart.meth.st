METHOD PUBLIC M_LocatePart : BOOL
VAR_INPUT
	iImageWidth: UINT:=2016;
	iImageHeight: UINT:=2000;
	uTimeout: UDINT:=60; //sec
	iMaxPos: UINT:=10; // i
	dMaxOverlap: REAL:=0.0; // j
	dScore: REAL:=0.8; // k
	dToleranceAngle: REAL:=90.0; // l
	sTemplateName: STRING:='template.jpg';
	sScriptFilename: STRING:='FastTemplateMatching.py';
END_VAR
VAR
	_Command : STRING(255);
END_VAR
VAR CONSTANT
	//_SCRIPT_FILE_NAME: STRING:='/var/opt/codesys/PlcLogic/Application/Vision/FastTemplateMatching.py';
	_SCRIPT_DIR_PATH: STRING:='/var/opt/codesys/PlcLogic/Application/Vision/';
	_RESULT_FILE_NAME: STRING:='/var/opt/codesys/PlcLogic/visu/outputimage.jpg';
	_TEMPLATE_DIR_PATH: STRING:='/var/opt/codesys/PlcLogic/Application/Vision/Templates/';
END_VAR

// run the designated vision program
// sudo python /var/opt/codesys/PlcLogic/Application/Vision/TemplateMatch.py -s /var/opt/codesys/PlcLogic/visu/outputimage.jpg        -t /var/opt/codesys/PlcLogic/Application/Vision/Template3.JPG -i 0.9 -j 1.0 -d true -w 2016 -h 2000
// sudo python /var/opt/codesys/PlcLogic/Application/Vision/FastTemplateMatching.py -s /var/opt/codesys/PlcLogic/visu/outputimage.jpg -t /var/opt/codesys/PlcLogic/Application/Vision/Template.jpg -w 2016 -h 2000 -d false
IF NOT _Busy THEN
	M_LocatePart := TRUE;
	
	_Command := CONCAT('sudo python ', _SCRIPT_DIR_PATH);
	_Command := CONCAT(_Command, sScriptFilename);
	//_Command := CONCAT(_Command, ' -s '); // need to trim down the string length
	//_Command := CONCAT(_Command, _VisionImageName);
	_Command := CONCAT(_Command, ' -t ');
	_Command := CONCAT(_Command, _TEMPLATE_DIR_PATH);
	_Command := CONCAT(_Command, sTemplateName);
	_Command := CONCAT(_Command, ' -i ');
	_Command := CONCAT(_Command, UINT_TO_STRING(iMaxPos));
	_Command := CONCAT(_Command, ' -j ');
	_Command := CONCAT(_Command, REAL_TO_STRING(dMaxOverlap));
	_Command := CONCAT(_Command, ' -k ');
	_Command := CONCAT(_Command, REAL_TO_STRING(dScore));
	_Command := CONCAT(_Command, ' -l ');
	_Command := CONCAT(_Command, REAL_TO_STRING(dToleranceAngle));
	_Command := CONCAT(_Command, ' -w ');
	_Command := CONCAT(_Command, UINT_TO_STRING(iImageWidth));
	_Command := CONCAT(_Command, ' -h ');
	_Command := CONCAT(_Command, UINT_TO_STRING(iImageHeight));
	_Command := CONCAT(_Command, ' 2> ./vision_log.txt ');
	
	SUPER^._M_Execute(Command:=_Command, Timeout:=uTimeout);
	
ELSE
	M_LocatePart := FALSE;
	//_Error := TRUE;
	//_ErrorDescription := 'function busy.';
	
END_IF
END_METHOD

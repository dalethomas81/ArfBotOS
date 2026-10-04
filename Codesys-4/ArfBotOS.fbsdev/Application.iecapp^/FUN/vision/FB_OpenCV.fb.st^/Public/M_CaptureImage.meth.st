METHOD PUBLIC M_CaptureImage : BOOL
VAR_INPUT
	iImageWidth: UINT:=2016;
	iImageHeight: UINT:=2000;
	uTimeout: UDINT:=60; //sec
END_VAR
VAR
	_Command : STRING(255);
END_VAR
VAR
	_ImageScriptFileName: STRING:='/var/opt/codesys/PlcLogic/Application/Vision/CaptureImage.py';
	_CaptureImageName: STRING:='/var/opt/codesys/PlcLogic/visu/outputimage.jpg';
END_VAR

// capture an image
// sudo python /var/opt/codesys/PlcLogic/Application/CaptureImage.py -s /var/opt/codesys/PlcLogic/Application/input.jpg -w 2016 -h 2000

IF NOT _Busy THEN
	M_CaptureImage := TRUE;
	
	_Command := CONCAT('sudo python ', _ImageScriptFileName);
	_Command := CONCAT(_Command, ' -s ');
	_Command := CONCAT(_Command, _CaptureImageName);
	_Command := CONCAT(_Command, ' -w ');
	_Command := CONCAT(_Command, UINT_TO_STRING(iImageWidth));
	_Command := CONCAT(_Command, ' -h ');
	_Command := CONCAT(_Command, UINT_TO_STRING(iImageHeight));
	_Command := CONCAT(_Command, ' 2> ./vision_log.txt ');
	
	SUPER^._M_Execute(Command:=_Command, Timeout:=uTimeout);
	
ELSE
	M_CaptureImage := FALSE;
	//_Error := TRUE;
	//_ErrorDescription := 'function busy.';
	
END_IF
END_METHOD

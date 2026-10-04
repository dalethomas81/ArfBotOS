METHOD PUBLIC M_CalibrateCamera : BOOL
VAR_INPUT
	iImageWidth: UINT:=2016;
	iImageHeight: UINT:=2000;
	uTimeout: UDINT:=60; //sec
	fCalCheckerSize: REAL:=25.0; //mm
	iCalCheckerX: UINT:=6;
	iCalCheckerY: UINT:=9;
END_VAR
VAR
	_Command : STRING(255);
END_VAR
VAR	
	_CalScriptFileName: STRING:='/var/opt/codesys/PlcLogic/Application/Vision/CalibrateCamera.py';
	_CalResultFile: STRING:='/var/opt/codesys/PlcLogic/visu/outputimage.jpg';
END_VAR

// calibrate with a checkerboard
// sudo python /var/opt/codesys/PlcLogic/Application/CalibrateCamera.py -s cal.yaml -l 24.5 -x 6 -y 9 -d false

IF NOT _Busy THEN
	M_CalibrateCamera := TRUE;
	
	_Command := CONCAT('sudo python ', _CalScriptFileName);
	_Command := CONCAT(_Command, ' -l ');
	_Command := CONCAT(_Command, REAL_TO_STRING(fCalCheckerSize));
	_Command := CONCAT(_Command, ' -x ');
	_Command := CONCAT(_Command, UINT_TO_STRING(iCalCheckerX));
	_Command := CONCAT(_Command, ' -y ');
	_Command := CONCAT(_Command, UINT_TO_STRING(iCalCheckerY));
	_Command := CONCAT(_Command, ' -w ');
	_Command := CONCAT(_Command, UINT_TO_STRING(iImageWidth));
	_Command := CONCAT(_Command, ' -h ');
	_Command := CONCAT(_Command, UINT_TO_STRING(iImageHeight));
	_Command := CONCAT(_Command, ' -r ');
	_Command := CONCAT(_Command, _CalResultFile);
	_Command := CONCAT(_Command, ' 2> ./vision_log.txt ');
	
	SUPER^._M_Execute(Command:=_Command, Timeout:=uTimeout);
	
ELSE
	M_CalibrateCamera := FALSE;
	//_Error := TRUE;
	//_ErrorDescription := 'function busy.';
	
END_IF
END_METHOD

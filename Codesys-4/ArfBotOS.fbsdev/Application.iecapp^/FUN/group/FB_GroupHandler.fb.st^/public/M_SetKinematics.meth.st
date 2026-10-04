METHOD PUBLIC M_SetKinematics : BOOL
VAR_INPUT
	CoordinateSystem: SMC_COORD_SYSTEM;
END_VAR

IF _GroupEnabled AND NOT Error THEN
	M_SetKinematics := TRUE;
	_CoordinateSystem := CoordinateSystem;
	_SetKinematics := TRUE;
	_KinematicsSet := FALSE;
ELSE
	M_SetKinematics := FALSE;
END_IF
END_METHOD

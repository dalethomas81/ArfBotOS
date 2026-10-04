METHOD PUBLIC M_Init : BOOL
VAR_INPUT
END_VAR
VAR
    // release
    _hRelease : SysTypes.RTS_IEC_HANDLE;
    _udiReleaseResult : SysTypes.RTS_IEC_RESULT;
    _udiReleaseRead : LWORD;
    _abyRelease : ARRAY[0..80] OF BYTE;
    _iRelease : INT;
    _nRelease : UDINT;
    _pReleaseLen : POINTER TO BYTE;
END_VAR

// get release tag to display on HMI
GVL_Version.sRelease := '';
_hRelease := SysFile.SysFileOpen(
    szFile := 'Application/RELEASE',
    am := SysFile.ACCESS_MODE.AM_READ,
    pResult := ADR(_udiReleaseResult));
IF _hRelease <> SysTypes.RTS_INVALID_HANDLE THEN
    _udiReleaseRead := SysFile.SysFileRead(
        hFile := _hRelease,
        pbyBuffer := ADR(_abyRelease),
        ulSize := SIZEOF(_abyRelease) - 1,
        pResult := ADR(_udiReleaseResult));
    SysFile.SysFileClose(_hRelease);

    _nRelease := LWORD_TO_UDINT(_udiReleaseRead);
    IF _nRelease > 80 THEN
        _nRelease := 80;
    END_IF
    WHILE _nRelease > 0 AND
        (_abyRelease[_nRelease - 1] = 10
        OR _abyRelease[_nRelease - 1] = 13
        OR _abyRelease[_nRelease - 1] = 0) DO
        _nRelease := _nRelease - 1;
    END_WHILE

    MemUtils.MemCpy(
        pbyDest := ADR(GVL_Version.sRelease),
        pbySrc := ADR(_abyRelease),
        dwSize := UDINT_TO_DWORD(_nRelease));
    _pReleaseLen := ADR(GVL_Version.sRelease) - 1;
    _pReleaseLen^ := UDINT_TO_BYTE(_nRelease);
    
    GVL_Version.sVersionLine := CONCAT(
        CONCAT('Installer ', GVL_Version.sRelease),
        CONCAT('  PLC ', GVL_Version.sPlcVersion));
    
END_IF
IF GVL_Version.sRelease = '' THEN
    GVL_Version.sVersionLine := 'unknown version';
    
END_IF

// check if arfbot has been initialized
IF NOT PersistentVars.Flags[GVL_App.INIT_POS_IDX].Value THEN
	// poses
	PersistentVars.Flags[GVL_App.HOM_TO_POSE_IDX].Value := FALSE;
	PersistentVars.StoredPositions[GVL_App.POSE_1_IDX].Position.X := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_1_IDX].Position.Y := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_1_IDX].Position.Z := -30.0;
	PersistentVars.StoredPositions[GVL_App.POSE_1_IDX].Position.A := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_1_IDX].Position.B := -30.0;
	PersistentVars.StoredPositions[GVL_App.POSE_1_IDX].Position.C := 0.0;
	
	PersistentVars.StoredPositions[GVL_App.POSE_2_IDX].Position.X := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_2_IDX].Position.Y := 30.0;
	PersistentVars.StoredPositions[GVL_App.POSE_2_IDX].Position.Z := -30.0;
	PersistentVars.StoredPositions[GVL_App.POSE_2_IDX].Position.A := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_2_IDX].Position.B := 30.0;
	PersistentVars.StoredPositions[GVL_App.POSE_2_IDX].Position.C := 0.0;
	
	PersistentVars.StoredPositions[GVL_App.POSE_3_IDX].Position.X := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_3_IDX].Position.Y := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_3_IDX].Position.Z := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_3_IDX].Position.A := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_3_IDX].Position.B := 90.0;
	PersistentVars.StoredPositions[GVL_App.POSE_3_IDX].Position.C := 0.0;
	
	PersistentVars.StoredPositions[GVL_App.POSE_4_IDX].Position.X := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_4_IDX].Position.Y := -4.74;
	PersistentVars.StoredPositions[GVL_App.POSE_4_IDX].Position.Z := -22.79;
	PersistentVars.StoredPositions[GVL_App.POSE_4_IDX].Position.A := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_4_IDX].Position.B := 62.45;
	PersistentVars.StoredPositions[GVL_App.POSE_4_IDX].Position.C := 0.0;
	
	PersistentVars.StoredPositions[GVL_App.POSE_5_IDX].Position.X := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_5_IDX].Position.Y := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_5_IDX].Position.Z := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_5_IDX].Position.A := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_5_IDX].Position.B := 0.0;
	PersistentVars.StoredPositions[GVL_App.POSE_5_IDX].Position.C := 0.0;
		 
	// decoder
	PersistentVars.StoredPositions[GVL_App.DEC_START_POS].Position.X := 300.0; // TODO need to verify all 6 of these
	PersistentVars.StoredPositions[GVL_App.DEC_START_POS].Position.Y := 0.0;
	PersistentVars.StoredPositions[GVL_App.DEC_START_POS].Position.Z := -400.0;
	PersistentVars.StoredPositions[GVL_App.DEC_START_POS].Position.A := 0.0;
	PersistentVars.StoredPositions[GVL_App.DEC_START_POS].Position.B := 0.0;
	PersistentVars.StoredPositions[GVL_App.DEC_START_POS].Position.C := 0.0;
	
	// set homing offsets
	PersistentVars.StoredPositions[GVL_App.HOM_POS_IDX].Position.X := 179.29; // J1 initial home offset
	PersistentVars.StoredPositions[GVL_App.HOM_POS_IDX].Position.Y := 36.74; // J2 initial home offset
	PersistentVars.StoredPositions[GVL_App.HOM_POS_IDX].Position.Z := -59.25; // J3 initial home offset
	PersistentVars.StoredPositions[GVL_App.HOM_POS_IDX].Position.A := -194.38; // J4 initial home offset
	PersistentVars.StoredPositions[GVL_App.HOM_POS_IDX].Position.B := -102.41; // J5 initial home offset
	PersistentVars.StoredPositions[GVL_App.HOM_POS_IDX].Position.C := 166.611; // J6 initial home offset
	
	// set speeds
	PersistentVars.StoredPositions[GVL_App.SPEEDS_IDX].Position.X := 100.0; // max velocity
	PersistentVars.StoredPositions[GVL_App.SPEEDS_IDX].Position.Y := 1000.0; // max accel/decel
	PersistentVars.StoredPositions[GVL_App.SPEEDS_IDX].Position.Z := 1000.0; // max jerk
	PersistentVars.StoredPositions[GVL_App.SPEEDS_IDX].Position.A := 1.0; // velocity factor
	PersistentVars.StoredPositions[GVL_App.SPEEDS_IDX].Position.B := 0.05; // accel/decel factor
	PersistentVars.StoredPositions[GVL_App.SPEEDS_IDX].Position.C := 1.0; // jerk factor
	
	//
	M_BuildTests();
		 
	// indicate initialized
	PersistentVars.Flags[GVL_App.INIT_POS_IDX].Value := TRUE;
END_IF

//
PersistentVars.StoredPositions[GVL_App.SPEEDS_IDX].Comment := 'vel, acc, jerk, vel fact, acc fact, jerk fact';
PersistentVars.StoredPositions[GVL_App.HOM_POS_IDX].Comment := 'Home Offsets J1, J2, J3, J4, J5, J6';
PersistentVars.StoredPositions[GVL_App.DEC_START_POS].Comment := 'NC Decoder Start Position';
PersistentVars.StoredPositions[GVL_App.TCP_POS_IDX].Comment := 'TCP Offset';
PersistentVars.StoredPositions[GVL_App.PC2_POS_IDX].Comment := 'PCS 2 Offset';
PersistentVars.StoredPositions[GVL_App.PC1_POS_IDX].Comment := 'PCS 1 Offset';
PersistentVars.StoredPositions[GVL_App.MCS_POS_IDX].Comment := 'MCS Offset';

//	 
PersistentVars.StoredPositions[GVL_App.POSE_1_IDX].Comment := 'Pose 1 coordinates';
PersistentVars.StoredPositions[GVL_App.POSE_2_IDX].Comment := 'Pose 2 coordinates';
PersistentVars.StoredPositions[GVL_App.POSE_3_IDX].Comment := 'Pose 3 coordinates';
PersistentVars.StoredPositions[GVL_App.POSE_4_IDX].Comment := 'Pose 4 coordinates';
PersistentVars.StoredPositions[GVL_App.POSE_5_IDX].Comment := 'Pose 5 coordinates';

//
PersistentVars.Flags[GVL_App.HOM_TO_POSE_IDX].Comment := 'Pose 2 After Home';

PersistentVars.Flags[GVL_App.INIT_DRV_IDX].Comment := 'Drives Initialized';
PersistentVars.Flags[GVL_App.INIT_DRV_IDX].Value := FALSE;

//
PersistentVars.Flags[GVL_App.HOME_COMPLETE].Comment := 'Axes Homed';
PersistentVars.Flags[GVL_App.HOME_COMPLETE].Value := FALSE;

//
PersistentVars.Flags[GVL_App.INIT_POS_IDX].Comment := 'Variables Intitialized';
END_METHOD

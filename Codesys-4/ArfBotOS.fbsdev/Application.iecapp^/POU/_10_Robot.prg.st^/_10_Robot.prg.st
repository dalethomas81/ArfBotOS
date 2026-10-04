PROGRAM _10_Robot
VAR	
	//
	AR4Inputs: ARRAY [0..31] OF BYTE;
	AR4Outputs: ARRAY [0..31] OF BYTE;
	
	//
	rx_heartbeat: BOOL;
	rx_en: BOOL;
	rx_j1_limit, rx_j2_limit, rx_j3_limit: BOOL;
	rx_j4_limit, rx_j5_limit, rx_j6_limit: BOOL;
	
	//
	InitializeDrives: BOOL;
	
	//
	FirstScan: BOOL:=TRUE;
	
END_VAR

//
IF FirstScan THEN
	M_Init(FALSE);
END_IF

_M_Init();
_M_HandleIO();

// do not put anything below this
FirstScan := FALSE;
END_PROGRAM

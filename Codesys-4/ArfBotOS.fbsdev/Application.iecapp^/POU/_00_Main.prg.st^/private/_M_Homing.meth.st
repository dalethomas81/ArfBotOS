METHOD PRIVATE _M_Homing : BOOL
VAR_INPUT
END_VAR

ProcessHome(
	J1:= IoConfig_Globals.J1, 
	J2:= IoConfig_Globals.J2, 
	J3:= IoConfig_Globals.J3, 
	J4:= IoConfig_Globals.J4, 
	J5:= IoConfig_Globals.J5, 
	J6:= IoConfig_Globals.J6, 
	AxisGroup:= ArfBotAxisGroup,
	GroupHandler:= GroupHandler, 
	Controller:= Controller, 
	J1_Limit:= _10_Robot.rx_j1_limit, 
	J2_Limit:= _10_Robot.rx_j2_limit, 
	J3_Limit:= _10_Robot.rx_j3_limit, 
	J4_Limit:= _10_Robot.rx_j4_limit, 
	J5_Limit:= _10_Robot.rx_j5_limit, 
	J6_Limit:= _10_Robot.rx_j6_limit, 
	Enable:= GVL_App.Enable, 
	Reset:= GVL_App.ResetRtrig.Q, 
	StoredPositions:= PersistentVars.StoredPositions,
	VisionRegisters:= PersistentVars.VisionRegisters,
	Flags:= PersistentVars.Flags,
	Done=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );

IF ProcessHome.Reset THEN
	HomingStatusText := '';
END_IF
IF ProcessHome.Error THEN
	JogStatusText := CONCAT('Homing Processor Error: ',
						CONCAT(TO_STRING(ProcessHome.ErrorID),
						CONCAT(' ', SMC_ErrorString(ProcessHome.ErrorID,
									SMC_Language_Type.english)))) ;
END_IF
END_METHOD

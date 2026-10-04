METHOD PRIVATE _M_Jogging : BOOL
VAR_INPUT
END_VAR
VAR_STAT
	EnableRtrig: R_TRIG;
	CoordSystemLast: SMC_COORD_SYSTEM;
	ABC_AS_ACSLast: BOOL;
END_VAR

// dont get sleepy
ProcessJog(
	AxisGroup:= ArfBotAxisGroup,
	GroupHandler:= GroupHandler,
	Controller:= Controller,
	Enable:= GVL_App.Enable,
	Reset:= GVL_App.ResetRtrig.Q,
	StoredPositions:= PersistentVars.StoredPositions,
	VisionRegisters:= PersistentVars.VisionRegisters,
	Flags:= PersistentVars.Flags,
	ABC_as_ACS=>,
	CoordSystem=>, 
	Enabled=>,
	Position=>, 
	Done=> , 
	Busy=> , 
	Error=> , 
	ErrorID=> );
	
// change the button texts on enable or a change of the coordinate system
EnableRtrig(CLK:=ProcessJog.CoordSystem <> CoordSystemLast
				OR ProcessJog.ABC_as_ACS <> ABC_AS_ACSLast);
IF EnableRtrig.Q THEN
	CoordSystemLast := ProcessJog.CoordSystem;
	ABC_AS_ACSLast := ProcessJog.ABC_as_ACS;
	ButtonTextIndex := F_SetJogButtons(ProcessJog.CoordSystem, ProcessJog.ABC_as_ACS);
END_IF
	
//
IF ProcessJog.Reset THEN
	JogStatusText := '';
END_IF
IF ProcessJog.Error THEN
	JogStatusText := CONCAT('Jog Processor Error: ',
						CONCAT(TO_STRING(ProcessJog.ErrorID),
						CONCAT(' ', SMC_ErrorString(ProcessJog.ErrorID,
									SMC_Language_Type.english)))) ;
END_IF
END_METHOD

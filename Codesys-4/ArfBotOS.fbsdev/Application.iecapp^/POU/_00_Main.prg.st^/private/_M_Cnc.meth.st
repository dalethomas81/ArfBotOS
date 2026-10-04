METHOD PRIVATE _M_Cnc : BOOL
VAR_INPUT
END_VAR
VAR
END_VAR

// track position of robot end effector to display on hmi
TrackPosition(
	AxisGroup:=ArfBotAxisGroup,
	J1:=IoConfig_Globals.J1,
	J2:=IoConfig_Globals.J2,
	J3:=IoConfig_Globals.J3,
	J4:=IoConfig_Globals.J4,
	J5:=IoConfig_Globals.J5,
	J6:=IoConfig_Globals.J6,
    OriConvention:= GVL_App.OriConvention,
	VisuStruct3DTrack=>,
	Arm=>,
	Elbow=>,
	Hand=>);
END_METHOD

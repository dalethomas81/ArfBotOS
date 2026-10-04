METHOD PRIVATE _M_GroupHandler : BOOL
VAR_INPUT
END_VAR

GroupHandler(
	AxisGroup:= ArfBotAxisGroup,
	Enable:= GVL_App.Enable,
	StoredPositions:= PersistentVars.StoredPositions,
	Busy=> , 
	Error=> , 
	ErrorID=> );
END_METHOD

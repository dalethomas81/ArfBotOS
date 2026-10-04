METHOD PRIVATE _M_HandlePose : BOOL
VAR_IN_OUT
	AxisGroup: AXIS_GROUP_REF_SM3;
	GroupHandler: FB_GroupHandler;
END_VAR
VAR_INPUT
END_VAR
VAR
	_MoveCommand: ST_MoveCmd;
	_PoseSpeed: LREAL:=20.0;
END_VAR

FOR _i:=0 TO GVL_App.POSE_5_IDX - GVL_App.POSE_1_IDX BY 1 DO
	IF _MoveToPose[_i] THEN
		_MoveToPose[_i] := FALSE;
		//
		IF GVL_App.Enable 
			AND_THEN PersistentVars.Flags[GVL_App.HOME_COMPLETE].Value
			AND_THEN NOT _MoveCommandProcessor.Busy
			AND_THEN NOT _MoveCommandProcessor.Error THEN
				//
				_MoveCommand.MoveType 			:= DUT_Command_MoveType.DirectMove;
				_MoveCommand.PositionType 		:= DUT_Command_PositionType.AbsolutePosition;
				_MoveCommand.CoordinateType 	:= DUT_Command_CoordinateType.Literal;
				_MoveCommand.Speed 				:= _PoseSpeed;
				_MoveCommand.CoordSystem 		:= SMC_COORD_SYSTEM.ACS;
				_MoveCommand.CoordRef 			:= PersistentVars.StoredPositions[GVL_App.POSE_1_IDX + _i].Position;
				//
				_MoveCommandProcessor.Command := F_BuildMoveCmd(_MoveCommand);
				_MoveCommandProcessor.M_Start();
				//
				EXIT;
		END_IF;
	END_IF
END_FOR

_MoveCommandProcessor(
	StoredPositions:= StoredPositions,
	VisionRegisters:= VisionRegisters,
	Flags:= Flags,
	AxisGroup:= AxisGroup, 
	GroupHandler:= GroupHandler,
	Velocity:= StoredPositions[GVL_App.SPEEDS_IDX].Position.X, 
	VelFactor:= StoredPositions[GVL_App.SPEEDS_IDX].Position.A,
	Acceleration:= StoredPositions[GVL_App.SPEEDS_IDX].Position.Y, 
	AccFactor:= StoredPositions[GVL_App.SPEEDS_IDX].Position.B,
	Jerk:= StoredPositions[GVL_App.SPEEDS_IDX].Position.Z,
	JerkFactor:= StoredPositions[GVL_App.SPEEDS_IDX].Position.C, 
	Position=> ,
	ItfUnitMode=> , 
	Error=> );
END_METHOD

FUNCTION_BLOCK FB_MoveCmdProcessor EXTENDS FB_Module
VAR_INPUT
	Velocity: LREAL:=1000.0;
	VelFactor: LREAL:=1.0;
	Acceleration: LREAL:=1000.0;
	AccFactor: LREAL:=0.5;
	Jerk: LREAL:=1000.0;
	JerkFactor: LREAL:=0.5;
	Command: STRING(GVL_App.COMMAND_LEN):='';
END_VAR
VAR_IN_OUT
	AxisGroup: AXIS_GROUP_REF_SM3;
	GroupHandler: FB_GroupHandler;
END_VAR
VAR_OUTPUT
	Position: SMC_POS_REF;
END_VAR
VAR
	_SequenceSetCoordTrans: INT;
	_SequenceMoveCircularAbsolute: INT;
	_SequenceMoveCircularRelative: INT;
	_SequenceMoveDirectAbsolute: INT;
	_SequenceMoveDirectRelative: INT;
	_SequenceMoveLinearAbsolute: INT;
	_SequenceMoveLinearRelative: INT;
	_MC_GroupHalt: MC_GroupHalt;
	
	_MC_MoveCircularAbsolute: MC_MoveCircularAbsolute;
	_MC_MoveCircularRelative: MC_MoveCircularRelative;
	_MC_MoveDirectAbsolute: MC_MoveDirectAbsolute;
	_MC_MoveDirectRelative: MC_MoveDirectRelative;
	_MC_MoveLinearAbsolute: MC_MoveLinearAbsolute;
	_MC_MoveLinearRelative: MC_MoveLinearRelative;
	
	_UnitModeCurrentState: PACK_ML.State;
	_MoveCommandParser: FB_ParseMoveCmd;
	_PosRef: SMC_POS_REF;
	_PosRef2: SMC_POS_REF;
	
END_VAR
VAR CONSTANT
	_STEP_DONE: 				INT := 1000;
	_STEP_IDLE: 				INT := 0000;
	_STEP_COMMAND: 				INT := 0020;
	_STEP_GROUP_POWER_ON: 		INT := 0022;
	_STEP_WAIT_POWER_ON: 		INT := 0024;
	_STEP_GROUP_ENABLE: 		INT := 0030;
	_STEP_WAIT_ENABLE: 			INT := 0035;
	_STEP_READ_POSITION: 		INT := 0040;
	_STEP_WAIT_POSITION: 		INT := 0050;
	_STEP_SET_KINEMATICS: 		INT := 0060;
	_STEP_WAIT_KINEMATICS: 		INT := 0070;
	_STEP_SET_COORD_TRANS:		INT := 0071;
	_STEP_WAIT_COORD_TRANS:		INT := 0072;
	_STEP_MOVE_TYPE:			INT := 0080;
	_STEP_WAIT_MOVE: 			INT := 0090;
	_STEP_GROUP_DISABLE: 		INT := 0100;
	_STEP_WAIT_DISABLE: 		INT := 0110;
	_STEP_GROUP_POWER_OFF: 		INT := 0120;
	_STEP_ERR: 					INT := 9999;
END_VAR

SUPER^();

// PackML state machine implementation
_UnitModeCurrentState := SUPER^._UnitMode.CurrentState;

//
CASE SUPER^._SeqMain OF
	
	_STEP_DONE:
		SUPER^._SeqMain := _STEP_IDLE;
		
	_STEP_IDLE:
		// wait here until commanded to start
		
	_STEP_COMMAND: // parse command
		_MoveCommandParser(Command:=Command);
		SUPER^._SeqMain := _STEP_GROUP_ENABLE;
		
	_STEP_GROUP_ENABLE:
		IF GroupHandler.M_EnableGroup() THEN
			SUPER^._SeqMain := _STEP_WAIT_ENABLE;
		ELSE
			SUPER^.M_Error(Description:=CONCAT('Group Handler Error: ',
										CONCAT(TO_STRING(GroupHandler.ErrorID),
										CONCAT(' ', SMC_ErrorString(GroupHandler.ErrorID,
													SMC_Language_Type.english)))));
			SUPER^._SeqMain := _STEP_ERR; // error step so we dont spam errors
		END_IF
		
	_STEP_WAIT_ENABLE:
		// TODO add timeout timer here?
		IF GroupHandler.P_GroupEnabled THEN
			SUPER^._SeqMain := _STEP_SET_KINEMATICS;
		END_IF;
		
	_STEP_SET_KINEMATICS: // set kinematics
        // orientation convention is always ZYZ for softmotion
        GVL_App.OriConvention := SMC_ORI_CONVENTION.ZYZ;
        //
		_PosRef.c := _MoveCommandParser.stCommand.CoordRef;
		_PosRef2.c := _MoveCommandParser.stCommand.CoordRef2;
		IF GroupHandler.M_SetKinematics(_MoveCommandParser.stCommand.CoordSystem) THEN
			SUPER^._SeqMain := _STEP_WAIT_KINEMATICS;
		ELSE
			SUPER^.M_Error(Description:=CONCAT('Group Handler Error: ',
										CONCAT(TO_STRING(GroupHandler.ErrorID),
										CONCAT(' ', SMC_ErrorString(GroupHandler.ErrorID,
													SMC_Language_Type.english)))));
			SUPER^._SeqMain := _STEP_ERR; // error step so we dont spam errors
		END_IF
		
	_STEP_WAIT_KINEMATICS: // wait for set kinematics
		// TODO add timeout timer here?
		IF GroupHandler.P_KinematicsSet THEN
			SUPER^._SeqMain := _STEP_MOVE_TYPE;
		END_IF
		
	_STEP_MOVE_TYPE: // parse move type
		IF _MoveCommandParser.stCommand.MoveType = DUT_Command_MoveType.CircularMove THEN
			IF _MoveCommandParser.stCommand.PositionType = DUT_Command_PositionType.AbsolutePosition THEN
				_SequenceMoveCircularAbsolute := 1;
			ELSIF _MoveCommandParser.stCommand.PositionType = DUT_Command_PositionType.RelativePosition THEN
				_SequenceMoveCircularRelative := 1;
			END_IF
			
		ELSIF _MoveCommandParser.stCommand.MoveType = DUT_Command_MoveType.DirectMove THEN
			IF _MoveCommandParser.stCommand.PositionType = DUT_Command_PositionType.AbsolutePosition THEN
				_SequenceMoveDirectAbsolute := 1;
			ELSIF _MoveCommandParser.stCommand.PositionType = DUT_Command_PositionType.RelativePosition THEN
				_SequenceMoveDirectRelative := 1;
			END_IF
			
		ELSIF _MoveCommandParser.stCommand.MoveType = DUT_Command_MoveType.LinearMove THEN
			IF _MoveCommandParser.stCommand.PositionType = DUT_Command_PositionType.AbsolutePosition THEN
				_SequenceMoveLinearAbsolute := 1;
			ELSIF _MoveCommandParser.stCommand.PositionType = DUT_Command_PositionType.RelativePosition THEN
				_SequenceMoveLinearRelative := 1;
			END_IF
		END_IF
		
		SUPER^._SeqMain := _STEP_WAIT_MOVE;
		
	_STEP_WAIT_MOVE: // wait for move complete
		IF _SequenceMoveCircularAbsolute = 0
			AND _SequenceMoveCircularRelative = 0 
			AND _SequenceMoveDirectAbsolute = 0 
			AND _SequenceMoveDirectRelative = 0
			AND _SequenceMoveLinearAbsolute = 0
			AND _SequenceMoveLinearRelative = 0 THEN
				//SUPER^._SeqMain := _STEP_GROUP_DISABLE;
				SUPER^._SeqMain := _STEP_DONE;
		END_IF
	
	// TODO do we need to disable the group at this point? lets leave this here for now but skip it.
	_STEP_GROUP_DISABLE: 
		IF GroupHandler.M_DisableGroup() THEN
			SUPER^._SeqMain := _STEP_WAIT_DISABLE;
		ELSE
			SUPER^.M_Error(Description:=CONCAT('Group Handler Error: ',
										CONCAT(TO_STRING(GroupHandler.ErrorID),
										CONCAT(' ', SMC_ErrorString(GroupHandler.ErrorID,
													SMC_Language_Type.english)))));
			SUPER^._SeqMain := _STEP_ERR; // error step so we dont spam errors
		END_IF
		
	_STEP_WAIT_DISABLE:
		IF NOT GroupHandler.P_GroupEnabled THEN
			SUPER^._SeqMain := _STEP_DONE;
		END_IF
	
END_CASE

CASE _SequenceMoveCircularAbsolute OF
	0: //
	
	1: //
		_MC_MoveCircularAbsolute.Execute := TRUE;
		_SequenceMoveCircularAbsolute := 2;
		
	2: //
		IF _MC_MoveCircularAbsolute.Error THEN
			SUPER^.M_Error(Description:=CONCAT('Move Command Error: ',
										CONCAT(TO_STRING(_MC_MoveCircularAbsolute.ErrorID),
										CONCAT(' ', SMC_ErrorString(_MC_MoveCircularAbsolute.ErrorID,
													SMC_Language_Type.english)))));
			_SequenceMoveCircularAbsolute := 3; // error step so we dont spam errors
		ELSIF  _MC_MoveCircularAbsolute.Done THEN
			_MC_MoveCircularAbsolute.Execute := FALSE;
			_SequenceMoveCircularAbsolute := 0;
		END_IF
		
END_CASE

CASE _SequenceMoveCircularRelative OF
	0: //
	
	1: //
		_MC_MoveCircularRelative.Execute := TRUE;
		_SequenceMoveCircularRelative := 2;
		
	2: //
		IF _MC_MoveCircularRelative.Error THEN
			SUPER^.M_Error(Description:=CONCAT('Move Command Error: ',
										CONCAT(TO_STRING(_MC_MoveCircularRelative.ErrorID),
										CONCAT(' ', SMC_ErrorString(_MC_MoveCircularRelative.ErrorID,
													SMC_Language_Type.english)))));
			_SequenceMoveCircularRelative := 3; // error step so we dont spam errors
		ELSIF  _MC_MoveCircularRelative.Done THEN
			_MC_MoveCircularRelative.Execute := FALSE;
			_SequenceMoveCircularRelative := 0;
		END_IF
		
END_CASE

CASE _SequenceMoveDirectAbsolute OF
	0: //
	
	1: //
		_MC_MoveDirectAbsolute.Execute := TRUE;
		_SequenceMoveDirectAbsolute := 2;
		
	2: //
		IF _MC_MoveDirectAbsolute.Error THEN
			SUPER^.M_Error(Description:=CONCAT('Move Command Error: ',
										CONCAT(TO_STRING(_MC_MoveDirectAbsolute.ErrorID),
										CONCAT(' ', SMC_ErrorString(_MC_MoveDirectAbsolute.ErrorID,
													SMC_Language_Type.english)))));
			_SequenceMoveDirectAbsolute := 3; // error step so we dont spam errors
			
		ELSIF _MC_MoveDirectAbsolute.Done THEN
			_MC_MoveDirectAbsolute.Execute := FALSE;
			_SequenceMoveDirectAbsolute := 0;
		END_IF
		
END_CASE

CASE _SequenceMoveDirectRelative OF
	0: //
	
	1: //
		_MC_MoveDirectRelative.Execute := TRUE;
		_SequenceMoveDirectRelative := 2;
		
	2: //
		IF _MC_MoveDirectRelative.Error THEN
			SUPER^.M_Error(Description:=CONCAT('Move Command Error: ',
										CONCAT(TO_STRING(_MC_MoveDirectRelative.ErrorID),
										CONCAT(' ', SMC_ErrorString(_MC_MoveDirectRelative.ErrorID,
													SMC_Language_Type.english)))));
			_SequenceMoveDirectRelative := 3; // error step so we dont spam errors
			
		ELSIF _MC_MoveDirectRelative.Done THEN
			_MC_MoveDirectRelative.Execute := FALSE;
			_SequenceMoveDirectRelative := 0;
		END_IF
		
END_CASE

CASE _SequenceMoveLinearAbsolute OF
	0: //
	
	1: //
		_MC_MoveLinearAbsolute.Execute := TRUE;
		_SequenceMoveLinearAbsolute := 2;
		
	2: //
		IF _MC_MoveLinearAbsolute.Error THEN
			SUPER^.M_Error(Description:=CONCAT('Move Command Error: ',
										CONCAT(TO_STRING(_MC_MoveLinearAbsolute.ErrorID),
										CONCAT(' ', SMC_ErrorString(_MC_MoveLinearAbsolute.ErrorID,
													SMC_Language_Type.english)))));
			_SequenceMoveLinearAbsolute := 3; // error step so we dont spam errors
			
		ELSIF _MC_MoveLinearAbsolute.Done THEN
			_MC_MoveLinearAbsolute.Execute := FALSE;
			_SequenceMoveLinearAbsolute := 0;
		END_IF
END_CASE

CASE _SequenceMoveLinearRelative OF
	0: //
	
	1: //
		_MC_MoveLinearRelative.Execute := TRUE;
		_SequenceMoveLinearRelative := 2;
		
	2: //
		IF _MC_MoveLinearRelative.Error THEN
		SUPER^.M_Error(Description:=CONCAT('Move Command Error: ',
										CONCAT(TO_STRING(_MC_MoveLinearRelative.ErrorID),
										CONCAT(' ', SMC_ErrorString(_MC_MoveLinearRelative.ErrorID,
													SMC_Language_Type.english)))));
			_SequenceMoveLinearRelative := 3; // error step so we dont spam errors
			
		ELSIF _MC_MoveLinearRelative.Done THEN
			_MC_MoveLinearRelative.Execute := FALSE;
			_SequenceMoveLinearRelative := 0;
		END_IF
		
END_CASE

_MC_GroupHalt(
	AxisGroup:= AxisGroup, 
	Execute:= , 
	Deceleration:= Acceleration, 
	Jerk:= Jerk, 
	AccFactor:= AccFactor, 
	JerkFactor:= JerkFactor, 
	TorqueFactor:= , 
	ClearMovements:= , 
	Done=> , 
	Busy=> , 
	Active=> , 
	CommandAborted=> , 
	CommandAccepted=> , 
	Error=> , 
	ErrorID=> , 
	MovementId=> );

// https://content.helpme-codesys.com/de/libs/SM3_Robotics/Current/SM3_Robotics/POUs/Movement/MC_MoveCircularAbsolute.html
// https://content.helpme-codesys.com/de/libs/SM3_Robotics/Current/SM3_Robotics/Basic-types/SMC_CIRC_MODE.html#smc-circ-mode
_MC_MoveCircularAbsolute(
	AxisGroup:= AxisGroup, 
	Execute:= , 
	CircMode:= _MoveCommandParser.stCommand.CircleMode, 
	AuxPoint:= _PosRef, 
	EndPoint:= _PosRef2, 
	PathChoice:= _MoveCommandParser.stCommand.PathChoice, 
	Velocity:= Velocity, 
	Acceleration:= Acceleration, 
	Deceleration:= Acceleration, 
	Jerk:= Jerk, 
	CoordSystem:= _MoveCommandParser.stCommand.CoordSystem, 
	BufferMode:= , 
	TransitionMode:= , 
	TransitionParameter:= , 
	OrientationMode:= SMC_ORIENTATION_MODE.Axis, 
	VelFactor:= SEL(_MoveCommandParser.stCommand.Speed > 100, _MoveCommandParser.stCommand.Speed / 100.0, 1), // cannot be greater than 1
	AccFactor:= SEL(AccFactor > 1, AccFactor, 1), 
	JerkFactor:= SEL(JerkFactor > 1, JerkFactor, 1), 
	TorqueFactor:= 1, 
	Done=> , 
	Busy=> , 
	Active=> , 
	CommandAborted=> , 
	CommandAccepted=> , 
	Error=> , 
	ErrorID=> , 
	MovementId=> );

// https://content.helpme-codesys.com/en/libs/SM3_Robotics/Current/SM3_Robotics/POUs/Movement/MC_MoveCircularRelative.html
// https://content.helpme-codesys.com/de/libs/SM3_Robotics/Current/SM3_Robotics/Basic-types/SMC_CIRC_MODE.html#smc-circ-mode
_MC_MoveCircularRelative(
	AxisGroup:= AxisGroup, 
	Execute:= , 
	CircMode:= _MoveCommandParser.stCommand.CircleMode, 
	AuxPoint:= _PosRef, 
	EndPoint:= _PosRef2, 
	PathChoice:= _MoveCommandParser.stCommand.PathChoice, 
	Velocity:= Velocity, 
	Acceleration:= Acceleration, 
	Deceleration:= Acceleration, 
	Jerk:= Jerk, 
	CoordSystem:= _MoveCommandParser.stCommand.CoordSystem, 
	BufferMode:= , 
	TransitionMode:= , 
	TransitionParameter:= , 
	OrientationMode:= SMC_ORIENTATION_MODE.Axis, 
	VelFactor:= SEL(_MoveCommandParser.stCommand.Speed > 100, _MoveCommandParser.stCommand.Speed / 100.0, 1), // cannot be greater than 1
	AccFactor:= SEL(AccFactor > 1, AccFactor, 1), 
	JerkFactor:= SEL(JerkFactor > 1, JerkFactor, 1), 
	TorqueFactor:= 1, 
	Done=> , 
	Busy=> , 
	Active=> , 
	CommandAborted=> , 
	CommandAccepted=> , 
	Error=> , 
	ErrorID=> , 
	MovementId=> );
_MC_MoveDirectAbsolute(
	AxisGroup:=AxisGroup,
	Execute:= ,
	Position:= _PosRef,
	MovementType:= ,
	CoordSystem:= _MoveCommandParser.stCommand.CoordSystem,
	BufferMode:= ,
	TransitionMode:= ,
	TransitionParameter:= ,
	VelFactor:= SEL(_MoveCommandParser.stCommand.Speed > 100, _MoveCommandParser.stCommand.Speed / 100.0, 1), // cannot be greater than 1
	AccFactor:= SEL(AccFactor > 1, AccFactor, 1), 
	JerkFactor:= SEL(JerkFactor > 1, JerkFactor, 1), 
	TorqueFactor:= 1,
	Done=> ,
	Busy=> ,
	Active=> ,
	CommandAborted=> ,
	CommandAccepted=> ,
	Error=> ,
	ErrorID=> ,
	MovementId=> );
_MC_MoveDirectRelative(
	AxisGroup:= AxisGroup, 
	Execute:= , 
	Distance:= _PosRef, 
	MovementType:= , 
	CoordSystem:= _MoveCommandParser.stCommand.CoordSystem, 
	BufferMode:= , 
	TransitionMode:= , 
	TransitionParameter:= , 
	VelFactor:= SEL(_MoveCommandParser.stCommand.Speed > 100, _MoveCommandParser.stCommand.Speed / 100.0, 1), // cannot be greater than 1
	AccFactor:= SEL(AccFactor > 1, AccFactor, 1), 
	JerkFactor:= SEL(JerkFactor > 1, JerkFactor, 1), 
	TorqueFactor:= 1, 
	Done=> , 
	Busy=> , 
	Active=> , 
	CommandAborted=> , 
	CommandAccepted=> , 
	Error=> , 
	ErrorID=> , 
	MovementId=> );
	
_MC_MoveLinearAbsolute(
	AxisGroup:= AxisGroup, 
	Execute:= , 
	Position:= _PosRef,
	Velocity:= Velocity, 
	Acceleration:= Acceleration, 
	Deceleration:= Acceleration, 
	Jerk:= Jerk, 
	CoordSystem:= _MoveCommandParser.stCommand.CoordSystem, 
	BufferMode:= , 
	TransitionMode:= , 
	TransitionParameter:= , 
	OrientationMode:= SMC_ORIENTATION_MODE.Axis, 
	VelFactor:= SEL(_MoveCommandParser.stCommand.Speed > 100, _MoveCommandParser.stCommand.Speed / 100.0, 1), // cannot be greater than 1
	AccFactor:= SEL(AccFactor > 1, AccFactor, 1), 
	JerkFactor:= SEL(JerkFactor > 1, JerkFactor, 1), 
	TorqueFactor:= 1, 
	Done=> , 
	Busy=> , 
	Active=> , 
	CommandAborted=> , 
	CommandAccepted=> , 
	Error=> , 
	ErrorID=> , 
	MovementId=> );
	
_MC_MoveLinearRelative(
	AxisGroup:= AxisGroup, 
	Execute:= , 
	Distance:= _PosRef, 
	Velocity:= Velocity, 
	Acceleration:= Acceleration, 
	Deceleration:= Acceleration, 
	Jerk:= Jerk, 
	CoordSystem:= _MoveCommandParser.stCommand.CoordSystem, 
	BufferMode:= , 
	TransitionMode:= , 
	TransitionParameter:= , 
	OrientationMode:= SMC_ORIENTATION_MODE.Axis, 
	VelFactor:= SEL(_MoveCommandParser.stCommand.Speed > 100, _MoveCommandParser.stCommand.Speed / 100.0, 1), // cannot be greater than 1
	AccFactor:= SEL(AccFactor > 1, AccFactor, 1), 
	JerkFactor:= SEL(JerkFactor > 1, JerkFactor, 1), 
	TorqueFactor:= 1, 
	Done=> , 
	Busy=> , 
	Active=> , 
	CommandAborted=> , 
	CommandAccepted=> , 
	Error=> , 
	ErrorID=> , 
	MovementId=> );
END_FUNCTION_BLOCK

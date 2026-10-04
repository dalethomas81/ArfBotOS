METHOD PRIVATE _M_HandleController : BOOL
VAR_IN_OUT
	Controller: FB_Controller;
END_VAR
VAR_INPUT
END_VAR
VAR_STAT
	OptionsTimer: TON:=(PT:=T#1S);
	OptionsRtrig: R_TRIG;
	ShareTimer: TON:=(PT:=T#1S);
	ShareRtrig: R_TRIG;
	OptionsShareRtrig: R_TRIG;
	JogEnabledRtrig: R_TRIG;
	JogEnabledFtrig: F_TRIG;
	SquareButtonRtrig: R_TRIG;
	TriangleButtonRtrig: R_TRIG;
	CircleButtonRtrig: R_TRIG;
	CrossButtonRtrig: R_TRIG;
	R2_Last: USINT;
	JoyStickLeftXLast: USINT;
	JoyStickLeftYLast: USINT;
	JoyStickRightXLast: USINT;
	JoyStickRightYLast: USINT;
	FactorMin: LREAL:=0.01;
END_VAR

//
OptionsTimer(IN:=Controller.P_State.Buttons.Options);
OptionsRtrig(CLK:=Controller.P_State.Buttons.Options);
//
ShareTimer(IN:=Controller.P_State.Buttons.Share);
ShareRtrig(CLK:=Controller.P_State.Buttons.Share);
//
OptionsShareRtrig(CLK:=OptionsTimer.Q AND_THEN ShareTimer.Q);
JogEnabledRtrig(CLK:=_SMC_GroupJog2.Enable);
JogEnabledFtrig(CLK:=_SMC_GroupJog2.Enable);


IF Enable AND_THEN Controller.P_State.Connected THEN
	
	// toggle jog enable
	IF OptionsShareRtrig.Q THEN
		_SMC_GroupJog2.Enable := NOT _SMC_GroupJog2.Enable;
	END_IF
	IF JogEnabledRtrig.Q OR JogEnabledFtrig.Q THEN
		IF _SMC_GroupJog2.Enable THEN
			_SMC_GroupJog2.CoordSystem := SMC_COORD_SYSTEM.TCS;
			Controller.M_SetTriggerMode(E_ControllerTrigger.Right, E_ControllerTriggerMode.Rigid);
			Controller.M_SetTriggerForce(E_ControllerTrigger.Right, E_ControllerTriggerForceParameter.One, 0);
			Controller.M_SetPlayerID(E_ControllerPlayerID.PlayerALL);
			Controller.M_SetMicrophoneState(TRUE);
		ELSE
			Controller.M_SetTriggerMode(E_ControllerTrigger.Right, E_ControllerTriggerMode.Off);
			Controller.M_SetMicrophoneState(FALSE);
		END_IF
	END_IF
	
	// cycle through coord system
	IF OptionsRtrig.Q AND NOT Controller.P_State.Buttons.Share THEN
		CASE _SMC_GroupJog2.CoordSystem OF
			SMC_COORD_SYSTEM.ACS:
				_SMC_GroupJog2.CoordSystem := SMC_COORD_SYSTEM.MCS;
				Controller.M_SetPlayerID(E_ControllerPlayerID.Player2);
			SMC_COORD_SYSTEM.MCS:
				_SMC_GroupJog2.CoordSystem := SMC_COORD_SYSTEM.PCS_1;
				Controller.M_SetPlayerID(E_ControllerPlayerID.Player3);
			SMC_COORD_SYSTEM.PCS_1:
				_SMC_GroupJog2.CoordSystem := SMC_COORD_SYSTEM.PCS_2;
				Controller.M_SetPlayerID(E_ControllerPlayerID.Player4);
			SMC_COORD_SYSTEM.PCS_2:
				_SMC_GroupJog2.CoordSystem := SMC_COORD_SYSTEM.TCS;
				Controller.M_SetPlayerID(E_ControllerPlayerID.PlayerALL);
			SMC_COORD_SYSTEM.TCS:
				_SMC_GroupJog2.CoordSystem := SMC_COORD_SYSTEM.ACS; // skip WCS
				Controller.M_SetPlayerID(E_ControllerPlayerID.Player1);
			SMC_COORD_SYSTEM.WCS:
				_SMC_GroupJog2.CoordSystem := SMC_COORD_SYSTEM.ACS;
				Controller.M_SetPlayerID(E_ControllerPlayerID.Player1);
		END_CASE
	END_IF
	
	// toggle the ABC as ACS setting
	IF ShareRtrig.Q AND NOT Controller.P_State.Buttons.Options THEN
		_SMC_GroupJog2.ABC_as_ACS := NOT _SMC_GroupJog2.ABC_as_ACS;
	END_IF
	
	// set the velocity factor while the left trigger button is pressed
	IF Controller.P_State.Dpad.Left AND_THEN Controller.P_State.Triggers.Right.Two <> R2_Last THEN
		R2_Last := Controller.P_State.Triggers.Right.Two;
		_SMC_GroupJog2.VelFactor := MAX(FactorMin, Controller.P_State.Triggers.Right.Two * 1.0 / 255.0);
	END_IF
	// set the acceleration factor while the up dpad button is pressed
	IF Controller.P_State.Dpad.Up AND_THEN Controller.P_State.Triggers.Right.Two <> R2_Last THEN
		R2_Last := Controller.P_State.Triggers.Right.Two;
		_SMC_GroupJog2.AccFactor := MAX(FactorMin, Controller.P_State.Triggers.Right.Two * 1.0 / 255.0);
	END_IF
	// set the jerk factor while the right dpad button is pressed
	IF Controller.P_State.Dpad.Right AND_THEN Controller.P_State.Triggers.Right.Two <> R2_Last THEN
		R2_Last := Controller.P_State.Triggers.Right.Two;
		_SMC_GroupJog2.JerkFactor := MAX(FactorMin, Controller.P_State.Triggers.Right.Two * 1.0 / 255.0);
	END_IF
	// set the torque factor while the down dpad down is pressed
	IF Controller.P_State.Dpad.Down AND_THEN Controller.P_State.Triggers.Right.Two <> R2_Last THEN
		R2_Last := Controller.P_State.Triggers.Right.Two;
		_SMC_GroupJog2.TorqueFactor := MAX(FactorMin, Controller.P_State.Triggers.Right.Two * 1.0 / 255.0);
	END_IF
	
	// first 4 poses can be called using buttons
	SquareButtonRtrig(CLK:=Controller.P_State.Buttons.Square);
	IF SquareButtonRtrig.Q THEN
		M_MoveToPose(E_Pose.One);
	END_IF
	TriangleButtonRtrig(CLK:=Controller.P_State.Buttons.Triangle);
	IF TriangleButtonRtrig.Q THEN
		M_MoveToPose(E_Pose.Two);
	END_IF
	CircleButtonRtrig(CLK:=Controller.P_State.Buttons.Circle);
	IF CircleButtonRtrig.Q THEN
		M_MoveToPose(E_Pose.Three);
	END_IF
	CrossButtonRtrig(CLK:=Controller.P_State.Buttons.Cross);
	IF CrossButtonRtrig.Q THEN
		M_MoveToPose(E_Pose.Four);
	END_IF
	
	//
	IF _SequenceMain = STEP_WAIT_JOG THEN
		
		//
		IF Controller.P_State.Joysticks.Left.X <> JoyStickLeftXLast
			OR Controller.P_State.Joysticks.Left.Y <> JoyStickLeftYLast THEN
				//
				JoyStickLeftXLast := Controller.P_State.Joysticks.Left.X;
				JoyStickLeftYLast := Controller.P_State.Joysticks.Left.Y;
				//	
				IF NOT Controller.P_State.Triggers.Left.One THEN
					// disable
					_SMC_GroupJog2.Forward[0] := FALSE;
					// disable
					_SMC_GroupJog2.Backward[0] := FALSE;
					// Controller.P_State.Joysticks.Left.X // X0-128 is -Y (Backward[1])
					_SMC_GroupJog2.Backward[1] := Controller.P_State.Joysticks.Left.X < _ControllerCenter - _ControllerDeadZone;
					// Controller.P_State.Joysticks.Left.X // X128-256 is +Y (Forward[1])
					_SMC_GroupJog2.Forward[1] := Controller.P_State.Joysticks.Left.X > _ControllerCenter + _ControllerDeadZone;
					// Controller.P_State.Joysticks.Left.Y // Y0-128 is +Z (Forward[2])
					_SMC_GroupJog2.Forward[2] := Controller.P_State.Joysticks.Left.Y < _ControllerCenter - _ControllerDeadZone;
					// Controller.P_State.Joysticks.Left.Y // Y128-256 is -Z (Backward[2])
					_SMC_GroupJog2.Backward[2] := Controller.P_State.Joysticks.Left.Y > _ControllerCenter + _ControllerDeadZone;
				ELSE
					// Controller.P_State.Joysticks.Left.Y // Y0-128 is +X (Forward[0])
					_SMC_GroupJog2.Forward[0] := Controller.P_State.Joysticks.Left.Y < _ControllerCenter - _ControllerDeadZone;
					// Controller.P_State.Joysticks.Left.Y // Y128-256 is -X (Backward[0])
					_SMC_GroupJog2.Backward[0] := Controller.P_State.Joysticks.Left.Y > _ControllerCenter + _ControllerDeadZone;
					// disable
					_SMC_GroupJog2.Backward[1] := FALSE;
					// disable
					_SMC_GroupJog2.Forward[1] := FALSE;
					// disable
					_SMC_GroupJog2.Forward[2] := FALSE;
					// disable])
					_SMC_GroupJog2.Backward[2] := FALSE;
				END_IF
		END_IF

		//
		IF Controller.P_State.Joysticks.Right.X <> JoyStickRightXLast
			OR Controller.P_State.Joysticks.Right.Y <> JoyStickRightYLast THEN
				//
				JoyStickRightXLast := Controller.P_State.Joysticks.Right.X;
				JoyStickRightYLast := Controller.P_State.Joysticks.Right.Y;
				//				
				IF NOT Controller.P_State.Triggers.Right.One THEN
					// Controller.P_State.Joysticks.Right.X // X0-128 is +Roll (rot about x) (Forward[3])
					_SMC_GroupJog2.Backward[3] := Controller.P_State.Joysticks.Right.X > _ControllerCenter + _ControllerDeadZone;
					// Controller.P_State.Joysticks.Right.X // X128-256 is -Roll (rot about x) (Backward[3])
					_SMC_GroupJog2.Forward[3] := Controller.P_State.Joysticks.Right.X < _ControllerCenter - _ControllerDeadZone;
					// Controller.P_State.Joysticks.Right.Y // Y0-128 is +Pitch (rot about y) (Forward[4])
					_SMC_GroupJog2.Forward[4] := Controller.P_State.Joysticks.Right.Y < _ControllerCenter - _ControllerDeadZone;
					// Controller.P_State.Joysticks.Right.Y // Y128-256 is -Pitch (rot about y) (Backward[4])
					_SMC_GroupJog2.Backward[4] := Controller.P_State.Joysticks.Right.Y > _ControllerCenter + _ControllerDeadZone;
					// disable
					_SMC_GroupJog2.Backward[5] := FALSE;
					// disable
					_SMC_GroupJog2.Forward[5] := FALSE;
				ELSE
					// disable
					_SMC_GroupJog2.Backward[3] := FALSE;
					// disable
					_SMC_GroupJog2.Forward[3] := FALSE;
					// disable
					_SMC_GroupJog2.Forward[4] := FALSE;
					// disable
					_SMC_GroupJog2.Backward[4] := FALSE;
					// Controller.P_State.Joysticks.Right.X // X0-128 is -Yaw (rot about z) (Backward[5])
					_SMC_GroupJog2.Backward[5] := Controller.P_State.Joysticks.Right.X < _ControllerCenter - _ControllerDeadZone;
					// Controller.P_State.Joysticks.Right.X // X128-256 is +Yaw (rot about z) (Forward[5])
					_SMC_GroupJog2.Forward[5] := Controller.P_State.Joysticks.Right.X > _ControllerCenter + _ControllerDeadZone;
				END_IF
		END_IF

	END_IF
	
END_IF
END_METHOD

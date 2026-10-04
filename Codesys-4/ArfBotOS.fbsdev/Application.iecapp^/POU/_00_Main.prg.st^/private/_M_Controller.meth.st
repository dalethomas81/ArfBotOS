METHOD PRIVATE _M_Controller : BOOL
VAR_INPUT
END_VAR
VAR_STAT
	_PowerRtrig: R_TRIG;
	_PowerFtrig: F_TRIG;
	_PowerTimerRtrig: R_TRIG;
	_PowerTimer: TON:=(PT:=T#1S);
	_PsButtonRtrig: R_TRIG;
	_PsButtonFtrig: F_TRIG;
	_PsButtonTimer: TON:=(PT:=T#2S);
	_PsButtonTimerRtrig: R_TRIG;
	_Reseting: BOOL;
END_VAR

// dont get sleepy
Controller(sIpAddress:= '127.0.0.1', uPort:= 50013);

// put two fingers on the trackpad for 3 seconds to toggle power
_PowerTimer(IN:=Controller.P_State.Connected
				AND Controller.P_State.Buttons.Touch
				AND Controller.P_State.Trackpads.One.Active 
				AND Controller.P_State.Trackpads.Two.Active);
_PowerTimerRtrig(CLK:=_PowerTimer.Q);
IF _PowerTimerRtrig.Q THEN
	GVL_App.PowerOn := NOT GVL_App.PowerOn;
END_IF
_PowerRtrig(CLK:=GVL_App.Enable);
_PowerFtrig(CLK:=GVL_App.Enable);
IF _PowerRtrig.Q OR _PowerFtrig.Q THEN
	IF GVL_App.Enable THEN
		Controller.M_SetColor(255,0,0);
		Controller.M_SetBrightness(E_ControllerBrightness.High);
		Controller.M_SetMotorIntensity(E_ControllerMotor.Left, 100, 0.100);
	ELSE
		Controller.M_SetColor(0,255,0);
		Controller.M_SetBrightness(E_ControllerBrightness.Low);
		Controller.M_SetMotorIntensity(E_ControllerMotor.Right, 100, 0.100);
	END_IF
END_IF

// press PS button once to reset
_PsButtonRtrig(CLK:=Controller.P_State.Buttons.PS);
_PsButtonFtrig(CLK:=Controller.P_State.Buttons.PS);
IF _PsButtonRtrig.Q THEN
	_Reseting := TRUE;
END_IF
IF _PsButtonFtrig.Q THEN
	IF _Reseting THEN //  reset was not overridden by holding it down too long
		_Reseting := FALSE;
		GVL_App.Reset := TRUE;
	END_IF
END_IF

// hold down the PS button to home
_PsButtonTimer(IN:=Controller.P_State.Buttons.PS);
_PsButtonTimerRtrig(CLK:=_PsButtonTimer.Q);
IF _PsButtonTimerRtrig.Q THEN
	_Reseting := FALSE; // ps button was held down too long so we dont reset
	ProcessHome.M_StartHoming();
END_IF
END_METHOD

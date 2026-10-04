METHOD PRIVATE _M_EvaluateControllerStates : BOOL
VAR_INPUT
	States: ARRAY[0..254] OF BYTE;
END_VAR
VAR_STAT
	_i, _j, _k: UINT;
	_FirstScan: BOOL:=TRUE;
	_HeartbeatTimer: TON:=(PT:=T#2S);
	_Heartbeat: USINT:=0;
	_HeartbeatLast: USINT:=0;
END_VAR
VAR CONSTANT
END_VAR

IF _FirstScan THEN
	//
END_IF

//
_Heartbeat := States[0] OR States[7]; // States[0] appears to be bluetooth and States[7] appears to be wired
IF _Heartbeat <> _HeartbeatLast THEN
	_HeartbeatLast := _Heartbeat;
	_HeartbeatTimer(IN:=FALSE);
ELSE
	_HeartbeatTimer(IN:=TRUE);
END_IF
Controller.Connected S= NOT _HeartbeatTimer.IN;
Controller.Connected R= _HeartbeatTimer.Q;

// clear all the data if the controller disconnects
IF NOT Controller.Connected THEN
	//MemUtils.MemSet(pbyBuffer:=ADR(States), byValue:=0, dwSize:=SIZEOF(States)); 
	FOR _i:=1 TO 254 BY 1 DO // start at 1 so we dont overwrite the heartbeat
		States[_i] := 0;
	END_FOR
END_IF

Controller.Joysticks.Left.X					:= States[1];
Controller.Joysticks.Left.Y					:= States[2];
Controller.Joysticks.Right.X				:= States[3];
Controller.Joysticks.Right.Y				:= States[4];
Controller.Triggers.Left.Two				:= States[5];
Controller.Triggers.Right.Two				:= States[6];

//UNUSED									:= States[7];

Controller.Buttons.Triangle 				:= States[8].7;
Controller.Buttons.Circle 					:= States[8].6;
Controller.Buttons.Cross 					:= States[8].5;
Controller.Buttons.Square 					:= States[8].4;
Controller.Dpad								:= F_DpadState(States[8] AND 16#0F);

Controller.Joysticks.Right.ThreeButton 		:= States[9].7;
Controller.Joysticks.Left.ThreeButton 		:= States[9].6;
Controller.Buttons.Options 					:= States[9].5;
Controller.Buttons.Share 					:= States[9].4;
Controller.Triggers.Right.TwoButton 		:= States[9].3;
Controller.Triggers.Left.TwoButton 			:= States[9].2;
Controller.Triggers.Right.One 				:= States[9].1;
Controller.Triggers.Left.One 				:= States[9].0;

Controller.Buttons.PS						:= States[10].0;
Controller.Buttons.Touch					:= States[10].1;
Controller.Buttons.Mic						:= States[10].2;

//UNUSED									:= States[11];
//UNUSED									:= States[12];
//UNUSED									:= States[13];
//UNUSED									:= States[14];
//UNUSED									:= States[15];

{warning disable C0308}
Controller.Accelerometer.X					:= SHL(States[16], 8) OR States[17];
Controller.Accelerometer.Y					:= SHL(States[18], 8) OR States[19];
Controller.Accelerometer.Z					:= SHL(States[20], 8) OR States[21];

Controller.Gyroscope.Pitch					:= SHL(States[22], 8) OR States[23];
Controller.Gyroscope.Yaw					:= SHL(States[24], 8) OR States[25];
Controller.Gyroscope.Roll					:= SHL(States[26], 8) OR States[27];

//UNUSED									:= States[28];
//UNUSED									:= States[29];
//UNUSED									:= States[30];
//UNUSED									:= States[31];
//UNUSED									:= States[32];

Controller.Trackpads.One.ID					:= States[33] AND 16#7F;
Controller.Trackpads.One.Active				:= (States[33] AND 16#80) = 0;
Controller.Trackpads.One.X					:= SHL((States[35] AND 16#0F), 8) OR States[34];
Controller.Trackpads.One.Y					:= SHL(States[36], 4) OR SHR((States[35] AND 16#F0), 4);
Controller.Trackpads.Two.ID					:= States[37] AND 16#7F;
Controller.Trackpads.Two.Active				:= (States[37] AND 16#80) = 0;
Controller.Trackpads.Two.X					:= SHL((States[39] AND 16#0F), 8) OR States[38];
Controller.Trackpads.Two.Y					:= SHL(States[40], 4) OR SHR((States[39] AND 16#F0), 4);
{warning restore C0308}

//UNUSED									:= States[41];
//UNUSED									:= States[42];
//UNUSED									:= States[43];
//UNUSED									:= States[44];
//UNUSED									:= States[45];
//UNUSED									:= States[46];
//UNUSED									:= States[47];
//UNUSED									:= States[48];
//UNUSED									:= States[49];
//UNUSED									:= States[50];
//UNUSED									:= States[51];
//UNUSED									:= States[52];

// from kit-nya
Controller.Battery.State					:= SHR(States[53] AND 16#F0, 4);
Controller.Battery.Level					:= MIN((States[53] AND 16#0F) * 10 + 5, 100); // TODO this will show a value of 5 when controller is disconnected.

// dont put anything below this
_FirstScan := FALSE;
END_METHOD

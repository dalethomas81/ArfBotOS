METHOD PRIVATE _M_HandleIO
VAR_INST
	//
	TxCrc, RxCrc: UINT;
	ChecksumMisCount: INT;
	
	//
	HeartbeatOn: TON;
	HeartbeatOff: TON;
	HeartbeatPt: TIME:=T#1000MS;
	
	//
	AR4OperationalFtrig: F_TRIG;
	
	//
	AllDrivesEnabled: BOOL;
	DriveEnable: ARRAY [0..5] OF BOOL;
	SetVelocity: ARRAY [0..5] OF INT;
	EncPosition: ARRAY [0..5] OF DINT;
	bufferIdx: INT;
	jointIdx: INT;
    
    //
    DeviceID: BYTE;
    RobotDeviceID: BYTE:=1;
	
END_VAR

//
HeartbeatOn(in:=NOT HeartbeatOff.Q,PT:=HeartbeatPt);
HeartbeatOff(in:=HeartbeatOn.Q,PT:=HeartbeatPt);

// if ethercat coms drop, clear the axes home status
{IF defined (IsSimulationMode)}
AR4OperationalFtrig(CLK:=TRUE);
{ELSE}
AR4OperationalFtrig(CLK:=AR4.wState = ETC_SLAVE_STATE.ETC_SLAVE_OPERATIONAL);
{END_IF}
IF AR4OperationalFtrig.Q THEN
	PersistentVars.Flags[GVL_App.HOME_COMPLETE].Value := FALSE;
	GVL_App.PowerOn := FALSE;
END_IF

// iSetVelocity is scaled counts, NOT °/s after the SoftMotion remap.
// deg/s = iSetVelocity * (MaxTU / MaxInt)
// Hz    = deg/s * StepsPerRev / 360

SetVelocity[0] := F_RoundToInt(
    (INT_TO_LREAL(IoConfig_Globals.J1.out.iSetVelocity) * GVL_App.c_PosCtrl_VelScale)
    * PersistentVars.PulsesPerRevolution[0] / 360.0);

SetVelocity[1] := F_RoundToInt(
    (INT_TO_LREAL(IoConfig_Globals.J2.out.iSetVelocity) * GVL_App.c_PosCtrl_VelScale)
    * PersistentVars.PulsesPerRevolution[1] / 360.0);

SetVelocity[2] := F_RoundToInt(
    (INT_TO_LREAL(IoConfig_Globals.J3.out.iSetVelocity) * GVL_App.c_PosCtrl_VelScale)
    * PersistentVars.PulsesPerRevolution[2] / 360.0);

SetVelocity[3] := F_RoundToInt(
    (INT_TO_LREAL(IoConfig_Globals.J4.out.iSetVelocity) * GVL_App.c_PosCtrl_VelScale)
    * PersistentVars.PulsesPerRevolution[3] / 360.0);

SetVelocity[4] := F_RoundToInt(
    (INT_TO_LREAL(IoConfig_Globals.J5.out.iSetVelocity) * GVL_App.c_PosCtrl_VelScale)
    * PersistentVars.PulsesPerRevolution[4] / 360.0);

SetVelocity[5] := F_RoundToInt(
    (INT_TO_LREAL(IoConfig_Globals.J6.out.iSetVelocity) * GVL_App.c_PosCtrl_VelScale)
    * PersistentVars.PulsesPerRevolution[5] / 360.0);

{region 'Output'}
// check that we are talking to the right device type
IF DeviceID = RobotDeviceID THEN// index 0 is for system outputs
    AR4Outputs[0].0:=HeartbeatOn.Q; // send a heartbeat to the drive
    AR4Outputs[0].1:=GVL_App.Enable; //
    // AR4Outputs[0].2:=; //
    // AR4Outputs[0].3:=; //
    // AR4Outputs[0].4:=; //
    // AR4Outputs[0].5:=; //
    // AR4Outputs[0].6:=; //
    // AR4Outputs[0].7:=; //
    
    // bytes 1 through 12 is for velocity to drive
    bufferIdx := 1;
    FOR jointIdx := 0 TO 5 BY 1 DO
        MEMUtils.MemCpy(pbyDest:=ADR(AR4Outputs[bufferIdx]), 
                        pbySrc:=ADR(SetVelocity[jointIdx]), 
                        dwSize:=SIZEOF(SetVelocity[jointIdx]));
        bufferIdx := bufferIdx + 2;
    END_FOR
    
    // bytes 13 through 24 are used for control from NC
    DriveEnable[0] := NOT (IoConfig_Globals.J1.nAxisState = SMC_AXIS_STATE.errorstop) AND NOT (IoConfig_Globals.J1.nAxisState = SMC_AXIS_STATE.power_off);
    DriveEnable[1] := NOT (IoConfig_Globals.J2.nAxisState = SMC_AXIS_STATE.errorstop) AND NOT (IoConfig_Globals.J2.nAxisState = SMC_AXIS_STATE.power_off);
    DriveEnable[2] := NOT (IoConfig_Globals.J3.nAxisState = SMC_AXIS_STATE.errorstop) AND NOT (IoConfig_Globals.J3.nAxisState = SMC_AXIS_STATE.power_off);
    DriveEnable[3] := NOT (IoConfig_Globals.J4.nAxisState = SMC_AXIS_STATE.errorstop) AND NOT (IoConfig_Globals.J4.nAxisState = SMC_AXIS_STATE.power_off);
    DriveEnable[4] := NOT (IoConfig_Globals.J5.nAxisState = SMC_AXIS_STATE.errorstop) AND NOT (IoConfig_Globals.J5.nAxisState = SMC_AXIS_STATE.power_off);
    DriveEnable[5] := NOT (IoConfig_Globals.J6.nAxisState = SMC_AXIS_STATE.errorstop) AND NOT (IoConfig_Globals.J6.nAxisState = SMC_AXIS_STATE.power_off);
    bufferIdx := 13;
    FOR jointIdx := 0 TO 5 BY 1 DO
        AR4Outputs[bufferIdx].3 := GVL_App.Enable AND DriveEnable[jointIdx];
        bufferIdx := bufferIdx + 1;
        // these are currently not used
        //AR4Outputs[bufferIdx].0 := Minus;
        //AR4Outputs[bufferIdx].1 := Plus;
        //AR4Outputs[bufferIdx].6 := Run;
        //AR4Outputs[bufferIdx].7 := Stop;
        bufferIdx := bufferIdx + 1;
    END_FOR
    
    // byte 25 is unused for now
    //AR4Outputs[25]:=;
    
    // byte 26 will be for gpio byte (bits 0 through 7)
    //AR4Outputs[26].0 := ;
    //AR4Outputs[26].1 := ;
    //AR4Outputs[26].2 := ;
    //AR4Outputs[26].3 := ;
    //AR4Outputs[26].4 := ;
    //AR4Outputs[26].5 := ;
    //AR4Outputs[26].6 := ;
    //AR4Outputs[26].7 := ;
    
    // bytes 27 through 28 are unused
    //AR4Outputs[27]:=;
    //AR4Outputs[28]:=;
    //AR4Outputs[29]:=;
    
    // byte 29 will be used to send the expected device id
    AR4Outputs[29] := RobotDeviceID;
    
END_IF

// bytes 30 and 31 will be used for crc
// calc crc here
TxCrc := F_Checksum(ADR(AR4Outputs), 30);
// append calc crc to tx buffer
MEMUtils.MemCpy(pbyDest:=ADR(AR4Outputs[30]), 
				pbySrc:=ADR(TxCrc), 
				dwSize:=2);
{endregion}
	
{region 'Input'}
// copy crc from rx buffer
MEMUtils.MemCpy(pbyDest:=ADR(RxCrc), 
				pbySrc:=ADR(AR4Inputs[30]), 
				dwSize:=2);

// compare calc crc with rx crc
{IF defined (IsSimulationMode)}
    PersistentVars.Flags[GVL_App.HOME_COMPLETE].Value := TRUE;
	rx_heartbeat := HeartbeatOn.Q;
	rx_en := GVL_App.Enable;
    EncPosition[0] := IoConfig_Globals.J1.diSetPosition;
    EncPosition[1] := IoConfig_Globals.J2.diSetPosition;
    EncPosition[2] := IoConfig_Globals.J3.diSetPosition;
    EncPosition[3] := IoConfig_Globals.J4.diSetPosition;
    EncPosition[4] := IoConfig_Globals.J5.diSetPosition;
    EncPosition[5] := IoConfig_Globals.J6.diSetPosition;
    rx_j1_limit :=
	rx_j2_limit :=
	rx_j3_limit :=
	rx_j4_limit :=
	rx_j5_limit :=
	rx_j6_limit :=
    IoConfig_Globals.J1.in.bDriveStartRealState :=
	IoConfig_Globals.J1.in.bRegulatorRealState :=
	IoConfig_Globals.J2.in.bDriveStartRealState :=
	IoConfig_Globals.J2.in.bRegulatorRealState :=
	IoConfig_Globals.J3.in.bDriveStartRealState :=
	IoConfig_Globals.J3.in.bRegulatorRealState :=
	IoConfig_Globals.J4.in.bDriveStartRealState :=
	IoConfig_Globals.J4.in.bRegulatorRealState :=
	IoConfig_Globals.J5.in.bDriveStartRealState :=
	IoConfig_Globals.J5.in.bRegulatorRealState :=
	IoConfig_Globals.J6.in.bDriveStartRealState :=
	IoConfig_Globals.J6.in.bRegulatorRealState :=
    AllDrivesEnabled := GVL_App.Enable;
    DeviceID := RobotDeviceID;
{ELSE}
IF RxCrc = F_Checksum(ADR(AR4Inputs), 30) THEN
	// byte 0 is for system inputs
	rx_heartbeat	:= AR4Inputs[0].0; // receive the heartbeat back
	rx_en 			:= AR4Inputs[0].1; // receive the enable back
	//rx_	 			:= AR4Inputs[0].2;
	//rx_ 				:= AR4Inputs[0].3;
	//rx_ 				:= AR4Inputs[0].4;
	//rx_ 				:= AR4Inputs[0].5;
	//rx_ 				:= AR4Inputs[0].6;
	//rx_ 				:= AR4Inputs[0].7;
	
	// bytes 1 through 24 is for encoder data from drive
	bufferIdx := 1;
	FOR jointIdx := 0 TO 5 BY 1 DO
		MEMUtils.MemCpy(pbyDest:=ADR(EncPosition[jointIdx]), 
						pbySrc:=ADR(AR4Inputs[bufferIdx]), 
						dwSize:=SIZEOF(EncPosition[jointIdx]));
		bufferIdx := bufferIdx + 4;
	END_FOR
	
	// byte 25 will be for limit switch inputs
	rx_j1_limit := AR4Inputs[25].0;
	rx_j2_limit := AR4Inputs[25].1;
	rx_j3_limit := AR4Inputs[25].2;
	rx_j4_limit := AR4Inputs[25].3;
	rx_j5_limit := AR4Inputs[25].4;
	rx_j6_limit := AR4Inputs[25].5;
	//rx_ := AR4Inputs[25].6;
	//rx_ := AR4Inputs[25].7;
	
	// byte 26 will be for gpio byte (bits 0 through 7)
	// := AR4Inputs[26].0;
	// := AR4Inputs[26].1;
	// := AR4Inputs[26].2;
	// := AR4Inputs[26].3;
	// := AR4Inputs[26].4;
	// := AR4Inputs[26].5;
	// := AR4Inputs[26].6;
	// := AR4Inputs[26].7;
	
	// byte 27 will be used to indicate drive is enabled
	IoConfig_Globals.J1.in.bDriveStartRealState := AR4Inputs[27].0;
	IoConfig_Globals.J1.in.bRegulatorRealState := AR4Inputs[27].0;
	IoConfig_Globals.J2.in.bDriveStartRealState := AR4Inputs[27].1;
	IoConfig_Globals.J2.in.bRegulatorRealState := AR4Inputs[27].1;
	IoConfig_Globals.J3.in.bDriveStartRealState := AR4Inputs[27].2;
	IoConfig_Globals.J3.in.bRegulatorRealState := AR4Inputs[27].2;
	IoConfig_Globals.J4.in.bDriveStartRealState := AR4Inputs[27].3;
	IoConfig_Globals.J4.in.bRegulatorRealState := AR4Inputs[27].3;
	IoConfig_Globals.J5.in.bDriveStartRealState := AR4Inputs[27].4;
	IoConfig_Globals.J5.in.bRegulatorRealState := AR4Inputs[27].4;
	IoConfig_Globals.J6.in.bDriveStartRealState := AR4Inputs[27].5;
	IoConfig_Globals.J6.in.bRegulatorRealState := AR4Inputs[27].5;
	// all drives are enabled
	AllDrivesEnabled := AR4Inputs[27].0 AND AR4Inputs[27].1 AND AR4Inputs[27].2
						AND AR4Inputs[27].3 AND AR4Inputs[27].4 AND AR4Inputs[27].5;
	
	// byte 28 is unused
	//
    
    // byte 29 will be used to identify the device
	DeviceID := AR4Inputs[29];
	
	// Rx buffer index 30 and 31 will be for crc
ELSE
	// increment crc mismatch counter
	ChecksumMisCount := ChecksumMisCount + 1;
END_IF
{END_IF}
{endregion}

// https://docs.google.com/spreadsheets/d/1S5TOxwbAx8pTMUdoWMnyC9WPfphhtUME/edit#gid=595805457
IoConfig_Globals.J1.in.dwActPosition := DINT_TO_DWORD(EncPosition[0]);
IoConfig_Globals.J2.in.dwActPosition := DINT_TO_DWORD(EncPosition[1]);
IoConfig_Globals.J3.in.dwActPosition := DINT_TO_DWORD(EncPosition[2]);
IoConfig_Globals.J4.in.dwActPosition := DINT_TO_DWORD(EncPosition[3]);
IoConfig_Globals.J5.in.dwActPosition := DINT_TO_DWORD(EncPosition[4]);
IoConfig_Globals.J6.in.dwActPosition := DINT_TO_DWORD(EncPosition[5]);

//
IF FirstScan THEN
	IoConfig_Globals.J1.bHWLimitEnable := TRUE;
	IoConfig_Globals.J2.bHWLimitEnable := TRUE;
	IoConfig_Globals.J3.bHWLimitEnable := TRUE;
	IoConfig_Globals.J4.bHWLimitEnable := TRUE;
	IoConfig_Globals.J5.bHWLimitEnable := TRUE;
	IoConfig_Globals.J6.bHWLimitEnable := TRUE;
END_IF

//
IoConfig_Globals.J1.in.bLimitNeg := TRUE;
IoConfig_Globals.J2.in.bLimitNeg := TRUE;
IoConfig_Globals.J3.in.bLimitNeg := rx_j3_limit;
IoConfig_Globals.J4.in.bLimitNeg := rx_j4_limit;
IoConfig_Globals.J5.in.bLimitNeg := rx_j5_limit;
IoConfig_Globals.J6.in.bLimitNeg := TRUE;

IoConfig_Globals.J1.in.bLimitPos := rx_j1_limit;
IoConfig_Globals.J2.in.bLimitPos := rx_j2_limit;
IoConfig_Globals.J3.in.bLimitPos := TRUE;
IoConfig_Globals.J4.in.bLimitPos := TRUE;
IoConfig_Globals.J5.in.bLimitPos := TRUE;
//IoConfig_Globals.J6.in.bLimitPos := rx_j6_limit;
// J6 home flag is not a travel stop — 360 deg tool-Z / cone orbits will cross it.
// Homing still uses rx_j6_limit as SMC_Homing.bReferenceSwitch (NOT J6_Limit).
IoConfig_Globals.J6.in.bLimitPos := TRUE;
END_METHOD
